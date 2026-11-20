use elan_0c4b_driver::matcher::{
    load_templates, save_templates, verify_against_templates, DEFAULT_MATCH_THRESHOLD,
};
use elan_0c4b_driver::{encode_pgm, normalize_u16_to_u8, subtract_background_u8, Elan0c4b};
use std::env;
use std::fs;
use std::path::PathBuf;
use std::process;

const ENROLL_SAMPLES: usize = 5;

fn print_usage(prog: &str) {
    eprintln!("Usage:");
    eprintln!("  {prog} info");
    eprintln!("  {prog} capture <output.pgm>");
    eprintln!("  {prog} capture-cal <output.pgm>");
    eprintln!("  {prog} enroll <finger-name>");
    eprintln!("  {prog} verify <finger-name> [threshold]");
}

fn template_path(finger: &str) -> PathBuf {
    let home = env::var("HOME").unwrap_or_else(|_| ".".to_string());
    PathBuf::from(home)
        .join(".local/share/elan-0c4b/templates")
        .join(format!("{finger}.bin"))
}

fn capture_calibrated_frame(dev: &Elan0c4b, width: u16, height: u16, bg: &[u16]) -> Vec<u8> {
    if let Err(e) = dev.wait_finger_press() {
        eprintln!("Wait finger failed: {e}");
        process::exit(1);
    }
    let samples = match dev.capture_raw_u16(width, height) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("Capture failed: {e}");
            process::exit(1);
        }
    };
    subtract_background_u8(&samples, bg)
}

fn main() {
    let args: Vec<String> = env::args().collect();
    let cmd = args.get(1).map(String::as_str);

    let dev = match Elan0c4b::open() {
        Ok(d) => d,
        Err(e) => {
            eprintln!("Failed to open 04f3:0c4b: {e}");
            eprintln!("Hint: check USB permissions (/dev/bus/usb/...) or install 99-elan-0c4b.rules.");
            process::exit(1);
        }
    };

    match cmd {
        Some("info") | None => match dev.info() {
            Ok(info) => {
                println!("Bridge FW Version : 0x{:04X}", info.fw_version);
                println!("Sensor Dimensions : {}x{}", info.width, info.height);
                println!("Sensor Status     : 0x{:02X}", info.status);
                if let Ok(cal) = dev.calibration_start() {
                    println!("Calibration Ready : 0x{:02X}", cal);
                }
                if let Ok(mean) = dev.calibration_mean() {
                    println!("Calibration Mean  : {mean}");
                }
            }
            Err(e) => {
                eprintln!("Error querying sensor info: {e}");
                process::exit(1);
            }
        },
        Some("capture") | Some("capture-cal") => {
            let calibrated = cmd == Some("capture-cal");
            let out_path = match args.get(2) {
                Some(p) => p,
                None => {
                    print_usage(&args[0]);
                    process::exit(2);
                }
            };
            let info = dev.info().unwrap_or_else(|e| {
                eprintln!("Error querying sensor info: {e}");
                process::exit(1);
            });

            let bg = if calibrated {
                println!("Capturing background calibration frame (keep finger off sensor)...");
                if let Err(e) = dev.calibration_start() {
                    eprintln!("Calibration start failed: {e}");
                    process::exit(1);
                }
                Some(dev.capture_raw_u16(info.width, info.height).unwrap_or_else(|e| {
                    eprintln!("Background capture failed: {e}");
                    process::exit(1);
                }))
            } else {
                None
            };

            println!(
                "Sensor {}x{} ready (FW 0x{:04X}). Place finger on sensor...",
                info.width, info.height, info.fw_version
            );
            if let Err(e) = dev.wait_finger_press() {
                eprintln!("Wait finger failed: {e}");
                process::exit(1);
            }
            println!("Finger detected, capturing raw frame...");
            let samples = dev.capture_raw_u16(info.width, info.height).unwrap_or_else(|e| {
                eprintln!("Capture failed: {e}");
                process::exit(1);
            });

            let pixels = match bg {
                Some(ref bg_frame) => subtract_background_u8(&samples, bg_frame),
                None => normalize_u16_to_u8(&samples),
            };
            let pgm = encode_pgm(info.width, info.height, &pixels);
            if let Err(e) = fs::write(out_path, pgm) {
                eprintln!("Failed to write {out_path}: {e}");
                process::exit(1);
            }
            println!("Saved {}x{} image to {out_path}", info.width, info.height);
        }
        Some("enroll") => {
            let finger = match args.get(2) {
                Some(f) => f,
                None => {
                    print_usage(&args[0]);
                    process::exit(2);
                }
            };
            let info = dev.info().unwrap_or_else(|e| {
                eprintln!("Error querying sensor info: {e}");
                process::exit(1);
            });

            println!("Calibrating background (keep finger off sensor)...");
            if let Err(e) = dev.calibration_start() {
                eprintln!("Calibration start failed: {e}");
                process::exit(1);
            }
            let bg = dev.capture_raw_u16(info.width, info.height).unwrap_or_else(|e| {
                eprintln!("Background capture failed: {e}");
                process::exit(1);
            });

            let mut templates = Vec::with_capacity(ENROLL_SAMPLES);
            for i in 1..=ENROLL_SAMPLES {
                println!("[{i}/{ENROLL_SAMPLES}] Touch the sensor with '{finger}'...");
                let frame = capture_calibrated_frame(&dev, info.width, info.height, &bg);
                templates.push(frame);
                println!("  Captured! Lift your finger...");
                let _ = dev.wait_finger_release();
            }

            let path = template_path(finger);
            if let Err(e) = save_templates(&path, info.width, info.height, &templates) {
                eprintln!("Failed to save templates: {e}");
                process::exit(1);
            }
            println!("Enrolled '{finger}' ({ENROLL_SAMPLES} samples) -> {}", path.display());
        }
        Some("verify") => {
            let finger = match args.get(2) {
                Some(f) => f,
                None => {
                    print_usage(&args[0]);
                    process::exit(2);
                }
            };
            let threshold: f32 = args
                .get(3)
                .and_then(|s| s.parse().ok())
                .unwrap_or(DEFAULT_MATCH_THRESHOLD);

            let path = template_path(finger);
            let (tw, th, templates) = load_templates(&path).unwrap_or_else(|e| {
                eprintln!("Failed to load templates from {}: {e}", path.display());
                process::exit(1);
            });

            let info = dev.info().unwrap_or_else(|e| {
                eprintln!("Error querying sensor info: {e}");
                process::exit(1);
            });
            if info.width != tw || info.height != th {
                eprintln!("Sensor dimensions {}x{} do not match template {}x{}", info.width, info.height, tw, th);
                process::exit(1);
            }

            println!("Calibrating background (keep finger off sensor)...");
            if let Err(e) = dev.calibration_start() {
                eprintln!("Calibration start failed: {e}");
                process::exit(1);
            }
            let bg = dev.capture_raw_u16(info.width, info.height).unwrap_or_else(|e| {
                eprintln!("Background capture failed: {e}");
                process::exit(1);
            });

            println!("Touch the sensor to verify '{finger}' (threshold {threshold:.2})...");
            let probe = capture_calibrated_frame(&dev, info.width, info.height, &bg);
            let score = verify_against_templates(
                usize::from(info.width),
                usize::from(info.height),
                &templates,
                &probe,
            );
            if score >= threshold {
                println!("MATCH (score: {score:.3} >= {threshold:.3})");
            } else {
                println!("NO MATCH (score: {score:.3} < {threshold:.3})");
                process::exit(1);
            }
        }
        _ => {
            print_usage(&args[0]);
            process::exit(2);
        }
    }
}
