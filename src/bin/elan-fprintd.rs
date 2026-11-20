//! Native Rust D-Bus daemon implementing `net.reactivated.Fprint.Manager` and
//! `net.reactivated.Fprint.Device` for the Elan `04f3:0c4b` fingerprint sensor.

use elan_0c4b_driver::matcher::{
    load_templates, save_templates, verify_prepared, MatchResult, PreparedTemplate,
    DEFAULT_MATCH_THRESHOLD, MIN_ADC_DIFF_SPAN,
};
use elan_0c4b_driver::{subtract_background_with_stats, Elan0c4b, SensorInfo};
use futures_util::StreamExt;
use std::collections::HashMap;
use std::env;
use std::ffi::CStr;
use std::fs;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};
use tokio::sync::Notify;
use zbus::fdo;
use zbus::message::Header;
use zbus::names::BusName;
use zbus::object_server::SignalContext;
use zbus::zvariant::ObjectPath;
use zbus::{interface, Connection, DBusError};

#[derive(Debug, DBusError)]
#[zbus(prefix = "net.reactivated.Fprint.Error")]
enum FprintError {
    #[zbus(error)]
    ZBus(zbus::Error),
    ClaimDevice(String),
    AlreadyInUse(String),
    Internal(String),
    NoEnrolledPrints(String),
    InvalidFingername(String),
}

const DEVICE_PATH: &str = "/net/reactivated/Fprint/Device/0";
const ENROLL_STAGES: i32 = 6;
const BG_CACHE_TTL: Duration = Duration::from_secs(30 * 60);
const BG_STALE_ON_FAIL: Duration = Duration::from_secs(5 * 60);
const MAX_VERIFY_QUALITY_RETRIES: u32 = 3;

const VALID_FINGERS: &[&str] = &[
    "left-thumb",
    "left-index-finger",
    "left-middle-finger",
    "left-ring-finger",
    "left-little-finger",
    "right-thumb",
    "right-index-finger",
    "right-middle-finger",
    "right-ring-finger",
    "right-little-finger",
];

#[derive(Clone)]
struct CachedSensor {
    info: SensorInfo,
    background: Vec<u16>,
    calibrated_at: Instant,
}

type TemplateCache = HashMap<(String, String), (u16, u16, Vec<PreparedTemplate>)>;

fn match_threshold() -> f32 {
    env::var("ELAN_MATCH_THRESHOLD")
        .ok()
        .and_then(|v| v.parse::<f32>().ok())
        .unwrap_or(DEFAULT_MATCH_THRESHOLD)
}

fn storage_root() -> PathBuf {
    if let Ok(dir) = env::var("ELAN_FPRINT_DIR") {
        return PathBuf::from(dir);
    }
    PathBuf::from("/var/lib/fprint/elan")
}

fn user_dir(username: &str) -> PathBuf {
    storage_root().join(username)
}

fn template_file(username: &str, finger: &str) -> PathBuf {
    user_dir(username).join(format!("{finger}.bin"))
}

fn list_user_fingers(username: &str) -> Vec<String> {
    let dir = user_dir(username);
    let Ok(entries) = fs::read_dir(dir) else {
        return Vec::new();
    };
    let mut fingers: Vec<String> = entries
        .flatten()
        .filter_map(|e| {
            let path = e.path();
            if path.extension().and_then(|s| s.to_str()) == Some("bin") {
                path.file_stem()
                    .and_then(|s| s.to_str())
                    .map(String::from)
            } else {
                None
            }
        })
        .collect();
    fingers.sort();
    fingers
}

fn uid_to_username(uid: u32) -> Option<String> {
    unsafe {
        let pw = libc::getpwuid(uid);
        if pw.is_null() || (*pw).pw_name.is_null() {
            return None;
        }
        Some(
            CStr::from_ptr((*pw).pw_name)
                .to_string_lossy()
                .into_owned(),
        )
    }
}

async fn resolve_username(
    conn: &Connection,
    header: &Header<'_>,
    requested: &str,
) -> Result<String, fdo::Error> {
    if !requested.is_empty() {
        if requested.contains('/') || requested.contains("..") {
            return Err(fdo::Error::InvalidArgs("invalid username".into()));
        }
        return Ok(requested.to_string());
    }
    let sender = header
        .sender()
        .ok_or_else(|| fdo::Error::Failed("missing D-Bus sender".into()))?;
    let proxy = fdo::DBusProxy::new(conn)
        .await
        .map_err(|e| fdo::Error::Failed(e.to_string()))?;
    let uid = proxy
        .get_connection_unix_user(sender.to_owned().into())
        .await
        .map_err(|e| fdo::Error::Failed(e.to_string()))?;
    uid_to_username(uid).ok_or_else(|| fdo::Error::Failed(format!("unknown uid {uid}")))
}

async fn sender_is_alive(conn: &Connection, sender: &str) -> bool {
    let Ok(proxy) = fdo::DBusProxy::new(conn).await else {
        return false;
    };
    let Ok(bus_name) = BusName::try_from(sender) else {
        return false;
    };
    proxy.name_has_owner(bus_name).await.unwrap_or(false)
}

#[derive(Default)]
struct DeviceState {
    claimed_by: Option<String>,
    claimed_sender: Option<String>,
    finger_present: bool,
    finger_needed: bool,
    active_cancel: Option<Arc<AtomicBool>>,
}

fn release_state(guard: &mut DeviceState) {
    if let Some(flag) = guard.active_cancel.take() {
        flag.store(true, Ordering::SeqCst);
    }
    guard.finger_needed = false;
    guard.finger_present = false;
    guard.claimed_by = None;
    guard.claimed_sender = None;
}

struct FprintManager;

#[interface(name = "net.reactivated.Fprint.Manager")]
impl FprintManager {
    async fn get_devices(&self) -> Vec<ObjectPath<'static>> {
        vec![ObjectPath::from_static_str_unchecked(DEVICE_PATH)]
    }

    async fn get_default_device(&self) -> fdo::Result<ObjectPath<'static>> {
        Ok(ObjectPath::from_static_str_unchecked(DEVICE_PATH))
    }
}

struct FprintDevice {
    conn: Connection,
    state: Arc<Mutex<DeviceState>>,
    usb_lock: Arc<tokio::sync::Mutex<()>>,
    sensor_cache: Arc<Mutex<Option<CachedSensor>>>,
    template_cache: Arc<Mutex<TemplateCache>>,
}

impl FprintDevice {
    fn ensure_claimed(&self) -> Result<String, FprintError> {
        let guard = self.state.lock().unwrap();
        guard
            .claimed_by
            .clone()
            .ok_or_else(|| FprintError::ClaimDevice("Device was not claimed".into()))
    }

    fn cancel_active(&self) {
        let mut guard = self.state.lock().unwrap();
        if let Some(flag) = guard.active_cancel.take() {
            flag.store(true, Ordering::SeqCst);
        }
        guard.finger_needed = false;
        guard.finger_present = false;
    }

    fn invalidate_user_cache(&self, user: &str, finger: Option<&str>) {
        let mut cache = self.template_cache.lock().unwrap();
        match finger {
            Some(f) => {
                cache.remove(&(user.to_string(), f.to_string()));
            }
            None => {
                cache.retain(|(u, _), _| u != user);
            }
        }
    }
}

fn get_or_refresh_sensor(
    dev: &Elan0c4b,
    sensor_cache: &Mutex<Option<CachedSensor>>,
    force_refresh: bool,
) -> elan_0c4b_driver::Result<CachedSensor> {
    if !force_refresh {
        let guard = sensor_cache.lock().unwrap();
        if let Some(cached) = guard.as_ref() {
            if cached.calibrated_at.elapsed() < BG_CACHE_TTL {
                return Ok(cached.clone());
            }
        }
    }
    let info = dev.info()?;
    let background = dev.capture_background(info.width, info.height)?;
    let fresh = CachedSensor {
        info,
        background,
        calibrated_at: Instant::now(),
    };
    *sensor_cache.lock().unwrap() = Some(fresh.clone());
    Ok(fresh)
}

fn load_prepared_templates(
    template_cache: &Mutex<TemplateCache>,
    user: &str,
    finger: &str,
    width: u16,
    height: u16,
) -> Option<Vec<PreparedTemplate>> {
    let key = (user.to_string(), finger.to_string());
    {
        let cache = template_cache.lock().unwrap();
        if let Some((tw, th, prepared)) = cache.get(&key) {
            if *tw == width && *th == height {
                return Some(prepared.clone());
            }
        }
    }

    let Ok((tw, th, frames)) = load_templates(&template_file(user, finger)) else {
        return None;
    };
    if tw != width || th != height {
        return None;
    }
    let prepared: Vec<PreparedTemplate> = frames
        .iter()
        .map(|f| PreparedTemplate::from_pixels(usize::from(tw), usize::from(th), f))
        .collect();
    template_cache
        .lock()
        .unwrap()
        .insert(key, (tw, th, prepared.clone()));
    Some(prepared)
}

#[interface(name = "net.reactivated.Fprint.Device")]
impl FprintDevice {
    #[zbus(property, name = "name")]
    async fn name(&self) -> String {
        "ElanTech 04f3:0c4b Fingerprint Sensor (Rust)".to_string()
    }

    #[zbus(property, name = "num-enroll-stages")]
    async fn num_enroll_stages(&self) -> i32 {
        ENROLL_STAGES
    }

    #[zbus(property, name = "scan-type")]
    async fn scan_type(&self) -> String {
        "press".to_string()
    }

    #[zbus(property, name = "finger-present")]
    async fn finger_present(&self) -> bool {
        self.state.lock().unwrap().finger_present
    }

    #[zbus(property, name = "finger-needed")]
    async fn finger_needed(&self) -> bool {
        self.state.lock().unwrap().finger_needed
    }

    async fn list_enrolled_fingers(
        &self,
        #[zbus(header)] header: Header<'_>,
        username: &str,
    ) -> Result<Vec<String>, FprintError> {
        let user = resolve_username(&self.conn, &header, username)
            .await
            .map_err(|e| FprintError::Internal(e.to_string()))?;
        let fingers = list_user_fingers(&user);
        if fingers.is_empty() {
            return Err(FprintError::NoEnrolledPrints(
                "No enrolled prints".into(),
            ));
        }
        Ok(fingers)
    }

    async fn delete_enrolled_fingers(
        &self,
        #[zbus(header)] header: Header<'_>,
        username: &str,
    ) -> fdo::Result<()> {
        let user = resolve_username(&self.conn, &header, username).await?;
        self.invalidate_user_cache(&user, None);
        let dir = user_dir(&user);
        if dir.exists() {
            let _ = fs::remove_dir_all(&dir);
        }
        Ok(())
    }

    async fn delete_enrolled_fingers2(&self) -> Result<(), FprintError> {
        let user = self.ensure_claimed()?;
        self.invalidate_user_cache(&user, None);
        let dir = user_dir(&user);
        if dir.exists() {
            let _ = fs::remove_dir_all(&dir);
        }
        Ok(())
    }

    async fn delete_enrolled_finger(&self, finger_name: &str) -> Result<(), FprintError> {
        let user = self.ensure_claimed()?;
        self.invalidate_user_cache(&user, Some(finger_name));
        let path = template_file(&user, finger_name);
        if path.exists() {
            fs::remove_file(&path).map_err(|e| FprintError::Internal(e.to_string()))?;
        }
        Ok(())
    }

    async fn claim(
        &self,
        #[zbus(header)] header: Header<'_>,
        username: &str,
    ) -> Result<(), FprintError> {
        let user = resolve_username(&self.conn, &header, username)
            .await
            .map_err(|e| FprintError::Internal(e.to_string()))?;
        let caller_sender = header.sender().map(|s| s.to_string());

        let existing = {
            let guard = self.state.lock().unwrap();
            (guard.claimed_by.is_some(), guard.claimed_sender.clone())
        };

        if existing.0 {
            let owner_alive = match existing.1.as_deref() {
                Some(owner) => sender_is_alive(&self.conn, owner).await,
                None => false,
            };
            if owner_alive {
                return Err(FprintError::AlreadyInUse("Device already claimed".into()));
            }
            eprintln!(
                "clearing stale claim from disconnected sender {:?}",
                existing.1
            );
            let mut guard = self.state.lock().unwrap();
            if guard.claimed_sender == existing.1 {
                release_state(&mut guard);
            }
        }

        let mut guard = self.state.lock().unwrap();
        if guard.claimed_by.is_some() {
            return Err(FprintError::AlreadyInUse("Device already claimed".into()));
        }
        guard.claimed_by = Some(user);
        guard.claimed_sender = caller_sender;
        Ok(())
    }

    async fn release(&self) -> Result<(), FprintError> {
        let mut guard = self.state.lock().unwrap();
        release_state(&mut guard);
        Ok(())
    }

    async fn verify_start(
        &self,
        #[zbus(signal_context)] ctxt: SignalContext<'_>,
        finger_name: &str,
    ) -> Result<(), FprintError> {
        let user = self.ensure_claimed()?;
        let available = list_user_fingers(&user);
        if available.is_empty() {
            return Err(FprintError::NoEnrolledPrints(
                "No enrolled prints".into(),
            ));
        }

        let selected: Vec<String> = if finger_name == "any" {
            available
        } else if VALID_FINGERS.contains(&finger_name) {
            if !template_file(&user, finger_name).exists() {
                return Err(FprintError::NoEnrolledPrints(
                    "No enrolled prints for finger".into(),
                ));
            }
            vec![finger_name.to_string()]
        } else {
            return Err(FprintError::InvalidFingername(
                "Invalid finger name".into(),
            ));
        };

        let first = selected[0].clone();
        Self::verify_finger_selected(&ctxt, &first)
            .await
            .map_err(|e| FprintError::Internal(e.to_string()))?;

        let cancel = Arc::new(AtomicBool::new(false));
        {
            let mut guard = self.state.lock().unwrap();
            if let Some(old) = guard.active_cancel.take() {
                old.store(true, Ordering::SeqCst);
            }
            guard.active_cancel = Some(cancel.clone());
            guard.finger_needed = true;
            guard.finger_present = false;
        }

        let state = self.state.clone();
        let usb_lock = self.usb_lock.clone();
        let sensor_cache = self.sensor_cache.clone();
        let template_cache = self.template_cache.clone();
        let ctxt = ctxt.to_owned();
        let threshold = match_threshold();

        tokio::spawn(async move {
            let _guard = usb_lock.lock().await;
            let (tx, mut rx) = tokio::sync::mpsc::unbounded_channel::<(&'static str, bool)>();

            let handle = tokio::task::spawn_blocking(move || {
                if cancel.load(Ordering::SeqCst) {
                    return;
                }
                let Ok(dev) = Elan0c4b::open() else {
                    let _ = tx.send(("verify-disconnected", true));
                    return;
                };
                let Ok(cached) = get_or_refresh_sensor(&dev, &sensor_cache, false) else {
                    let _ = tx.send(("verify-unknown-error", true));
                    return;
                };
                let info = cached.info;
                let bg = cached.background;

                let mut prepared_by_finger = Vec::with_capacity(selected.len());
                for finger in &selected {
                    if let Some(prep) =
                        load_prepared_templates(&template_cache, &user, finger, info.width, info.height)
                    {
                        prepared_by_finger.push((finger.clone(), prep));
                    }
                }
                if prepared_by_finger.is_empty() {
                    let _ = tx.send(("verify-unknown-error", true));
                    return;
                }

                for attempt in 0..=MAX_VERIFY_QUALITY_RETRIES {
                    match dev.wait_finger_press_cancellable(&cancel, Duration::from_secs(20)) {
                        Ok(true) => {}
                        Ok(false) => {
                            if !cancel.load(Ordering::SeqCst) {
                                let _ = tx.send(("verify-no-match", true));
                            }
                            return;
                        }
                        Err(e) => {
                            eprintln!("verify wait_finger_press error: {e}");
                            let _ = tx.send(("verify-unknown-error", true));
                            return;
                        }
                    }

                    if cancel.load(Ordering::SeqCst) {
                        return;
                    }

                    let raw = match dev.capture_raw_u16(info.width, info.height) {
                        Ok(r) => r,
                        Err(e) => {
                            eprintln!("verify capture_raw_u16 error: {e}");
                            let _ = tx.send(("verify-unknown-error", true));
                            return;
                        }
                    };
                    let (probe_u8, span) = subtract_background_with_stats(&raw, &bg);
                    let probe_tmpl = PreparedTemplate::from_pixels(
                        usize::from(info.width),
                        usize::from(info.height),
                        &probe_u8,
                    );

                    if (span < MIN_ADC_DIFF_SPAN || !probe_tmpl.is_good_quality())
                        && attempt < MAX_VERIFY_QUALITY_RETRIES
                    {
                        eprintln!(
                            "verify low-quality touch user={user} attempt={} span={span} fg={:.0}%",
                            attempt + 1,
                            probe_tmpl.foreground_ratio * 100.0
                        );
                        let _ = tx.send(("verify-retry-scan", false));
                        let _ =
                            dev.wait_finger_release_cancellable(&cancel, Duration::from_secs(5));
                        continue;
                    }

                    let match_start = Instant::now();
                    let mut best_finger = prepared_by_finger[0].0.as_str();
                    let mut best = MatchResult {
                        score: 0.0,
                        angle_deg: 0.0,
                        dx: 0,
                        dy: 0,
                    };

                    for (finger, templates) in &prepared_by_finger {
                        let res = verify_prepared(templates, &probe_tmpl, Some(threshold));
                        if res.score > best.score {
                            best = res;
                            best_finger = finger.as_str();
                        }
                        if best.score >= threshold {
                            break;
                        }
                    }

                    let match_ms = match_start.elapsed().as_secs_f32() * 1000.0;
                    eprintln!(
                        "verify user={user} finger={best_finger} score={:.3} angle={:.0}deg dx={} dy={} fg={:.0}% match_ms={:.2}",
                        best.score,
                        best.angle_deg,
                        best.dx,
                        best.dy,
                        probe_tmpl.foreground_ratio * 100.0,
                        match_ms
                    );

                    if best.score >= threshold {
                        let _ = tx.send(("verify-match", true));
                    } else {
                        if cached.calibrated_at.elapsed() >= BG_STALE_ON_FAIL {
                            *sensor_cache.lock().unwrap() = None;
                        }
                        let _ = tx.send(("verify-no-match", true));
                    }
                    return;
                }

                let _ = tx.send(("verify-no-match", true));
            });

            while let Some((msg, done)) = rx.recv().await {
                let _ = Self::verify_status(&ctxt, msg, done).await;
                if done {
                    break;
                }
            }
            let _ = handle.await;

            let mut g = state.lock().unwrap();
            g.finger_needed = false;
            g.finger_present = false;
            g.active_cancel = None;
        });

        Ok(())
    }

    async fn verify_stop(&self) -> fdo::Result<()> {
        self.cancel_active();
        Ok(())
    }

    async fn enroll_start(
        &self,
        #[zbus(signal_context)] ctxt: SignalContext<'_>,
        finger_name: &str,
    ) -> Result<(), FprintError> {
        let user = self.ensure_claimed()?;
        if !VALID_FINGERS.contains(&finger_name) {
            return Err(FprintError::InvalidFingername(
                "Invalid finger name".into(),
            ));
        }

        let cancel = Arc::new(AtomicBool::new(false));
        {
            let mut guard = self.state.lock().unwrap();
            if let Some(old) = guard.active_cancel.take() {
                old.store(true, Ordering::SeqCst);
            }
            guard.active_cancel = Some(cancel.clone());
            guard.finger_needed = true;
            guard.finger_present = false;
        }

        let finger = finger_name.to_string();
        let state = self.state.clone();
        let usb_lock = self.usb_lock.clone();
        let sensor_cache = self.sensor_cache.clone();
        let template_cache = self.template_cache.clone();
        let ctxt = ctxt.to_owned();

        tokio::spawn(async move {
            let _guard = usb_lock.lock().await;
            let (tx, mut rx) = tokio::sync::mpsc::unbounded_channel::<(&'static str, bool)>();

            let handle = tokio::task::spawn_blocking(move || {
                let Ok(dev) = Elan0c4b::open() else {
                    let _ = tx.send(("enroll-disconnected", true));
                    return;
                };
                let Ok(cached) = get_or_refresh_sensor(&dev, &sensor_cache, true) else {
                    let _ = tx.send(("enroll-failed", true));
                    return;
                };
                let info = cached.info;
                let bg = cached.background;

                let mut samples = Vec::with_capacity(ENROLL_STAGES as usize);
                let mut prepared_samples = Vec::with_capacity(ENROLL_STAGES as usize);
                let mut retries = 0u32;
                while samples.len() < ENROLL_STAGES as usize {
                    if cancel.load(Ordering::SeqCst) {
                        let _ = tx.send(("enroll-failed", true));
                        return;
                    }
                    match dev.wait_finger_press_cancellable(&cancel, Duration::from_secs(20)) {
                        Ok(true) => {}
                        Ok(false) => {
                            if cancel.load(Ordering::SeqCst) {
                                let _ = tx.send(("enroll-failed", true));
                                return;
                            }
                            retries += 1;
                            if retries > 5 {
                                let _ = tx.send(("enroll-failed", true));
                                return;
                            }
                            let _ = tx.send(("enroll-retry-scan", false));
                            continue;
                        }
                        Err(e) => {
                            eprintln!("enroll wait_finger_press error: {e}");
                            retries += 1;
                            if retries > 5 {
                                let _ = tx.send(("enroll-failed", true));
                                return;
                            }
                            let _ = tx.send(("enroll-retry-scan", false));
                            continue;
                        }
                    }

                    let raw = match dev.capture_raw_u16(info.width, info.height) {
                        Ok(r) => r,
                        Err(e) => {
                            eprintln!("enroll capture_raw_u16 error: {e}");
                            retries += 1;
                            if retries > 5 {
                                let _ = tx.send(("enroll-failed", true));
                                return;
                            }
                            let _ = tx.send(("enroll-retry-scan", false));
                            continue;
                        }
                    };

                    let (frame, span) = subtract_background_with_stats(&raw, &bg);
                    let prep = PreparedTemplate::from_pixels(
                        usize::from(info.width),
                        usize::from(info.height),
                        &frame,
                    );
                    if span < MIN_ADC_DIFF_SPAN || !prep.is_good_quality() {
                        eprintln!(
                            "enroll low-quality touch user={user} finger={finger} span={span} fg={:.0}%",
                            prep.foreground_ratio * 100.0
                        );
                        retries += 1;
                        if retries > 8 {
                            let _ = tx.send(("enroll-failed", true));
                            return;
                        }
                        let _ = tx.send(("enroll-retry-scan", false));
                        let _ =
                            dev.wait_finger_release_cancellable(&cancel, Duration::from_secs(5));
                        continue;
                    }

                    retries = 0;
                    samples.push(frame);
                    prepared_samples.push(prep);
                    let _ = dev.wait_finger_release_cancellable(&cancel, Duration::from_secs(5));
                    if samples.len() < ENROLL_STAGES as usize {
                        let _ = tx.send(("enroll-stage-passed", false));
                    }
                }

                let path = template_file(&user, &finger);
                match save_templates(&path, info.width, info.height, &samples) {
                    Ok(()) => {
                        template_cache.lock().unwrap().insert(
                            (user.clone(), finger.clone()),
                            (info.width, info.height, prepared_samples),
                        );
                        let _ = tx.send(("enroll-completed", true));
                    }
                    Err(e) => {
                        eprintln!("enroll save_templates({}) failed: {e}", path.display());
                        let _ = tx.send(("enroll-failed", true));
                    }
                }
            });

            while let Some((msg, done)) = rx.recv().await {
                let _ = Self::enroll_status(&ctxt, msg, done).await;
                if done {
                    break;
                }
            }
            let _ = handle.await;

            let mut g = state.lock().unwrap();
            g.finger_needed = false;
            g.finger_present = false;
            g.active_cancel = None;
        });

        Ok(())
    }

    async fn enroll_stop(&self) -> fdo::Result<()> {
        self.cancel_active();
        Ok(())
    }

    #[zbus(signal)]
    async fn verify_finger_selected(ctxt: &SignalContext<'_>, finger_name: &str) -> zbus::Result<()>;

    #[zbus(signal)]
    async fn verify_status(
        ctxt: &SignalContext<'_>,
        result: &str,
        done: bool,
    ) -> zbus::Result<()>;

    #[zbus(signal)]
    async fn enroll_status(
        ctxt: &SignalContext<'_>,
        result: &str,
        done: bool,
    ) -> zbus::Result<()>;
}

#[tokio::main]
async fn main() -> zbus::Result<()> {
    let conn = Connection::system().await?;
    let usb_lock = Arc::new(tokio::sync::Mutex::new(()));
    let state = Arc::new(Mutex::new(DeviceState::default()));
    let sensor_cache = Arc::new(Mutex::new(None));
    let template_cache = Arc::new(Mutex::new(HashMap::new()));

    let device = FprintDevice {
        conn: conn.clone(),
        state: state.clone(),
        usb_lock: usb_lock.clone(),
        sensor_cache: sensor_cache.clone(),
        template_cache,
    };

    conn.object_server()
        .at("/net/reactivated/Fprint/Manager", FprintManager)
        .await?;
    conn.object_server().at(DEVICE_PATH, device).await?;
    conn.request_name("net.reactivated.Fprint").await?;

    // Watch D-Bus disconnects so stale claims from crashed/killed clients auto-release.
    let watch_conn = conn.clone();
    let watch_state = state;
    tokio::spawn(async move {
        if let Ok(proxy) = fdo::DBusProxy::new(&watch_conn).await {
            if let Ok(mut stream) = proxy.receive_name_owner_changed().await {
                while let Some(signal) = stream.next().await {
                    if let Ok(args) = signal.args() {
                        if args.new_owner.is_none() {
                            let mut guard = watch_state.lock().unwrap();
                            if guard.claimed_sender.as_deref() == Some(args.name.as_str()) {
                                eprintln!(
                                    "claiming client {} disconnected, auto-releasing device",
                                    args.name
                                );
                                release_state(&mut guard);
                            }
                        }
                    }
                }
            }
        }
    });

    // Pre-calibrate sensor background on startup so first verification is immediate.
    tokio::spawn(async move {
        let _guard = usb_lock.lock().await;
        let _ = tokio::task::spawn_blocking(move || match Elan0c4b::open() {
            Ok(dev) => match get_or_refresh_sensor(&dev, &sensor_cache, true) {
                Ok(cached) => {
                    eprintln!(
                        "startup calibration ready: fw=0x{:04X} {}x{}",
                        cached.info.fw_version, cached.info.width, cached.info.height
                    );
                }
                Err(e) => eprintln!("startup calibration failed: {e}"),
            },
            Err(e) => eprintln!("startup sensor open failed: {e}"),
        })
        .await;
    });

    // Run until terminated by systemd/SIGINT
    let notify = Notify::new();
    notify.notified().await;
    Ok(())
}
