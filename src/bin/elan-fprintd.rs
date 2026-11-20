//! Native Rust D-Bus daemon implementing `net.reactivated.Fprint.Manager` and
//! `net.reactivated.Fprint.Device` for the Elan `04f3:0c4b` fingerprint sensor.

use elan_0c4b_driver::matcher::{
    load_templates, save_templates, verify_against_templates, DEFAULT_MATCH_THRESHOLD,
};
use elan_0c4b_driver::{subtract_background_u8, Elan0c4b};
use std::env;
use std::ffi::CStr;
use std::fs;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use tokio::sync::Notify;
use zbus::fdo;
use zbus::message::Header;
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
const ENROLL_STAGES: i32 = 5;

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

#[derive(Default)]
struct DeviceState {
    claimed_by: Option<String>,
    finger_present: bool,
    finger_needed: bool,
    active_cancel: Option<Arc<AtomicBool>>,
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
        let dir = user_dir(&user);
        if dir.exists() {
            let _ = fs::remove_dir_all(&dir);
        }
        Ok(())
    }

    async fn delete_enrolled_fingers2(&self) -> Result<(), FprintError> {
        let user = self.ensure_claimed()?;
        let dir = user_dir(&user);
        if dir.exists() {
            let _ = fs::remove_dir_all(&dir);
        }
        Ok(())
    }

    async fn delete_enrolled_finger(&self, finger_name: &str) -> Result<(), FprintError> {
        let user = self.ensure_claimed()?;
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
        let mut guard = self.state.lock().unwrap();
        if guard.claimed_by.is_some() {
            return Err(FprintError::AlreadyInUse("Device already claimed".into()));
        }
        guard.claimed_by = Some(user);
        Ok(())
    }

    async fn release(&self) -> Result<(), FprintError> {
        self.cancel_active();
        let mut guard = self.state.lock().unwrap();
        guard.claimed_by = None;
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
        let ctxt = ctxt.to_owned();

        tokio::spawn(async move {
            let _guard = usb_lock.lock().await;
            let result = tokio::task::spawn_blocking(move || {
                if cancel.load(Ordering::SeqCst) {
                    return ("verify-no-match", true);
                }
                let Ok(dev) = Elan0c4b::open() else {
                    return ("verify-disconnected", true);
                };
                let Ok(info) = dev.info() else {
                    return ("verify-unknown-error", true);
                };
                let _ = dev.calibration_start();
                let Ok(bg) = dev.capture_raw_u16(info.width, info.height) else {
                    return ("verify-unknown-error", true);
                };
                if cancel.load(Ordering::SeqCst) {
                    return ("verify-no-match", true);
                }
                if dev.wait_finger_press().is_err() {
                    return ("verify-unknown-error", true);
                }
                if cancel.load(Ordering::SeqCst) {
                    return ("verify-no-match", true);
                }
                let Ok(raw) = dev.capture_raw_u16(info.width, info.height) else {
                    return ("verify-unknown-error", true);
                };
                let probe = subtract_background_u8(&raw, &bg);

                let mut best_score = 0.0f32;
                for finger in &selected {
                    if let Ok((tw, th, templates)) = load_templates(&template_file(&user, finger)) {
                        if tw == info.width && th == info.height {
                            let s = verify_against_templates(
                                usize::from(info.width),
                                usize::from(info.height),
                                &templates,
                                &probe,
                            );
                            if s > best_score {
                                best_score = s;
                            }
                        }
                    }
                }
                if best_score >= DEFAULT_MATCH_THRESHOLD {
                    ("verify-match", true)
                } else {
                    ("verify-no-match", true)
                }
            })
            .await
            .unwrap_or(("verify-unknown-error", true));

            {
                let mut g = state.lock().unwrap();
                g.finger_needed = false;
                g.finger_present = false;
                g.active_cancel = None;
            }
            let _ = Self::verify_status(&ctxt, result.0, result.1).await;
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
        let ctxt = ctxt.to_owned();

        tokio::spawn(async move {
            let _guard = usb_lock.lock().await;
            let (tx, mut rx) = tokio::sync::mpsc::unbounded_channel::<(&'static str, bool)>();

            let handle = tokio::task::spawn_blocking(move || {
                let Ok(dev) = Elan0c4b::open() else {
                    let _ = tx.send(("enroll-disconnected", true));
                    return;
                };
                let Ok(info) = dev.info() else {
                    let _ = tx.send(("enroll-failed", true));
                    return;
                };
                let _ = dev.calibration_start();
                let Ok(bg) = dev.capture_raw_u16(info.width, info.height) else {
                    let _ = tx.send(("enroll-failed", true));
                    return;
                };

                let mut samples = Vec::with_capacity(ENROLL_STAGES as usize);
                let mut retries = 0u32;
                while samples.len() < ENROLL_STAGES as usize {
                    if cancel.load(Ordering::SeqCst) {
                        let _ = tx.send(("enroll-failed", true));
                        return;
                    }
                    if let Err(e) = dev.wait_finger_press() {
                        eprintln!("enroll wait_finger_press error: {e}");
                        retries += 1;
                        if retries > 5 {
                            let _ = tx.send(("enroll-failed", true));
                            return;
                        }
                        let _ = tx.send(("enroll-retry-scan", false));
                        continue;
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
                    retries = 0;
                    samples.push(subtract_background_u8(&raw, &bg));
                    let _ = dev.wait_finger_release();
                    if samples.len() < ENROLL_STAGES as usize {
                        let _ = tx.send(("enroll-stage-passed", false));
                    }
                }

                let path = template_file(&user, &finger);
                match save_templates(&path, info.width, info.height, &samples) {
                    Ok(()) => {
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

    let device = FprintDevice {
        conn: conn.clone(),
        state,
        usb_lock,
    };

    conn.object_server()
        .at("/net/reactivated/Fprint/Manager", FprintManager)
        .await?;
    conn.object_server().at(DEVICE_PATH, device).await?;
    conn.request_name("net.reactivated.Fprint").await?;

    // Run until terminated by systemd/SIGINT
    let notify = Notify::new();
    notify.notified().await;
    Ok(())
}
