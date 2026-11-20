//! Userspace USB driver for the Elan Microelectronics `04f3:0c4b` fingerprint sensor.
//!
//! Protocol constants and pipe assignments were reverse-engineered from `WbfUsbDriver.dll`:
//! - `EP_CMD_OUT` (`0x01`): Bulk OUT for commands (`SetCmdRequest`)
//! - `EP_INFO_IN` (`0x83`): Bulk IN for command/register responses (`GetCmdRequest`)
//! - `EP_IMG_IN`  (`0x82`): Bulk IN for raw image data (`SendReadRequest`)

pub mod matcher;

use rusb::{Context, DeviceHandle, UsbContext};
use std::fmt;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

pub const ELAN_VID: u16 = 0x04f3;
pub const ELAN_PID: u16 = 0x0c4b;

pub const EP_CMD_OUT: u8 = 0x01;
pub const EP_INFO_IN: u8 = 0x83;
pub const EP_IMG_IN: u8 = 0x82;

pub const CMD_TIMEOUT: Duration = Duration::from_millis(500);
pub const WAIT_FINGER_TIMEOUT: Duration = Duration::from_secs(15);
pub const IMG_TIMEOUT: Duration = Duration::from_millis(2000);
const POLL_SLICE: Duration = Duration::from_millis(200);

#[derive(Debug)]
pub enum DriverError {
    DeviceNotFound,
    Usb(rusb::Error),
    ShortWrite { expected: usize, actual: usize },
    ShortRead { expected: usize, actual: usize },
    UnexpectedResponse { cmd: &'static str, data: Vec<u8> },
}

impl fmt::Display for DriverError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::DeviceNotFound => write!(f, "Elan 04f3:0c4b device not found"),
            Self::Usb(e) => write!(f, "USB error: {e}"),
            Self::ShortWrite { expected, actual } => {
                write!(f, "short USB write: expected {expected} bytes, wrote {actual}")
            }
            Self::ShortRead { expected, actual } => {
                write!(f, "short USB read: expected {expected} bytes, read {actual}")
            }
            Self::UnexpectedResponse { cmd, data } => {
                write!(f, "unexpected response for {cmd}: {data:02X?}")
            }
        }
    }
}

impl std::error::Error for DriverError {}

impl From<rusb::Error> for DriverError {
    fn from(err: rusb::Error) -> Self {
        Self::Usb(err)
    }
}

pub type Result<T> = std::result::Result<T, DriverError>;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SensorInfo {
    pub fw_version: u16,
    pub width: u16,
    pub height: u16,
    pub status: u8,
}

pub struct Elan0c4b {
    handle: DeviceHandle<Context>,
}

impl Elan0c4b {
    /// Open the first `04f3:0c4b` USB device and claim interface 0.
    pub fn open() -> Result<Self> {
        let ctx = Context::new()?;
        let handle = ctx
            .open_device_with_vid_pid(ELAN_VID, ELAN_PID)
            .ok_or(DriverError::DeviceNotFound)?;

        if handle.kernel_driver_active(0).unwrap_or(false) {
            handle.detach_kernel_driver(0)?;
        }
        handle.claim_interface(0)?;
        Ok(Self { handle })
    }

    fn write_cmd(&self, cmd: &[u8]) -> Result<()> {
        let written = self.handle.write_bulk(EP_CMD_OUT, cmd, CMD_TIMEOUT)?;
        if written != cmd.len() {
            return Err(DriverError::ShortWrite {
                expected: cmd.len(),
                actual: written,
            });
        }
        Ok(())
    }

    fn read_info(&self, cmd: &[u8], len: usize, timeout: Duration) -> Result<Vec<u8>> {
        self.write_cmd(cmd)?;
        let mut buf = vec![0u8; len];
        let read = self.handle.read_bulk(EP_INFO_IN, &mut buf, timeout)?;
        buf.truncate(read);
        Ok(buf)
    }

    /// `GetBridgeFWVersion` (`0x40 0x19` -> 2-byte big-endian u16).
    pub fn bridge_fw_version(&self) -> Result<u16> {
        let buf = self.read_info(&[0x40, 0x19], 2, CMD_TIMEOUT)?;
        decode_fw_version(&buf)
    }

    /// `ReadSensorWidth` (`0x40 0x21 0x09` -> `buf[0] + 1`).
    pub fn sensor_width(&self) -> Result<u16> {
        let buf = self.read_info(&[0x40, 0x21, 0x09], 2, CMD_TIMEOUT)?;
        decode_dimension(&buf)
    }

    /// `ReadSensorHeight` (`0x40 0x21 0x08` -> `buf[0] + 1`).
    pub fn sensor_height(&self) -> Result<u16> {
        let buf = self.read_info(&[0x40, 0x21, 0x08], 2, CMD_TIMEOUT)?;
        decode_dimension(&buf)
    }

    /// `ReadSensorStatus` (`0x40 0x13` -> `buf[0]`).
    pub fn sensor_status(&self) -> Result<u8> {
        let buf = self.read_info(&[0x40, 0x13], 2, CMD_TIMEOUT)?;
        if buf.is_empty() {
            return Err(DriverError::ShortRead {
                expected: 1,
                actual: 0,
            });
        }
        Ok(buf[0])
    }

    /// `SetBridgeToCalibrationStart` (`0x40 0x23` -> retries up to 5 times for `0x03`).
    pub fn calibration_start(&self) -> Result<u8> {
        let mut last = 0u8;
        for _ in 0..5 {
            let buf = self.read_info(&[0x40, 0x23], 2, CMD_TIMEOUT)?;
            if let Some(&v) = buf.first() {
                last = v;
                if v == 0x03 {
                    return Ok(v);
                }
            }
        }
        Ok(last)
    }

    /// `GetBridgeCalibrationMean` (`0x40 0x24` -> 2-byte big-endian u16).
    pub fn calibration_mean(&self) -> Result<u16> {
        let buf = self.read_info(&[0x40, 0x24], 2, CMD_TIMEOUT)?;
        decode_fw_version(&buf)
    }

    /// Query firmware version, dimensions, and status.
    pub fn info(&self) -> Result<SensorInfo> {
        Ok(SensorInfo {
            fw_version: self.bridge_fw_version()?,
            width: self.sensor_width()?,
            height: self.sensor_height()?,
            status: self.sensor_status()?,
        })
    }

    /// `SetBridgeCancel` (`0x00 0x0B` from `WbfUsbDriver.dll@0x180009F10`),
    /// then drain any trailing response on `EP_INFO_IN`.
    pub fn bridge_cancel(&self) -> Result<()> {
        self.write_cmd(&[0x00, 0x0b])?;
        let mut drain = [0u8; 64];
        let _ = self
            .handle
            .read_bulk(EP_INFO_IN, &mut drain, Duration::from_millis(50));
        Ok(())
    }

    /// `SensorWaitFingerPress` (`0x40 0x3F`).
    pub fn wait_finger_press(&self) -> Result<Vec<u8>> {
        self.read_info(&[0x40, 0x3f], 2, WAIT_FINGER_TIMEOUT)
    }

    /// Poll `SensorWaitFingerPress` (`0x40 0x3F`) in 200ms slices so cancellation
    /// aborts quickly via `bridge_cancel`.
    pub fn wait_finger_press_cancellable(
        &self,
        cancel: &AtomicBool,
        max_wait: Duration,
    ) -> Result<bool> {
        self.write_cmd(&[0x40, 0x3f])?;
        let deadline = Instant::now() + max_wait;
        let mut buf = [0u8; 64];

        loop {
            if cancel.load(Ordering::SeqCst) || Instant::now() >= deadline {
                let _ = self.bridge_cancel();
                return Ok(false);
            }
            match self.handle.read_bulk(EP_INFO_IN, &mut buf, POLL_SLICE) {
                Ok(n) if n > 0 => {
                    if buf[..n].contains(&0xff) {
                        return Ok(false);
                    }
                    return Ok(true);
                }
                Ok(_) => {}
                Err(rusb::Error::Timeout) => {
                    if cancel.load(Ordering::SeqCst) || Instant::now() >= deadline {
                        let _ = self.bridge_cancel();
                        return Ok(false);
                    }
                }
                Err(e) => return Err(e.into()),
            }
        }
    }

    /// `SensorWaitFingerRelease` (`0x40 0x3E` -> expects `0xAA` in response).
    pub fn wait_finger_release(&self) -> Result<()> {
        let buf = self.read_info(&[0x40, 0x3e], 256, WAIT_FINGER_TIMEOUT)?;
        if buf.contains(&0xaa) {
            return Ok(());
        }
        Err(DriverError::UnexpectedResponse {
            cmd: "SensorWaitFingerRelease",
            data: buf,
        })
    }

    /// Poll `SensorWaitFingerRelease` (`0x40 0x3E`) in 200ms slices so cancellation
    /// aborts quickly via `bridge_cancel`.
    pub fn wait_finger_release_cancellable(
        &self,
        cancel: &AtomicBool,
        max_wait: Duration,
    ) -> Result<bool> {
        self.write_cmd(&[0x40, 0x3e])?;
        let deadline = Instant::now() + max_wait;
        let mut buf = [0u8; 256];

        loop {
            if cancel.load(Ordering::SeqCst) || Instant::now() >= deadline {
                let _ = self.bridge_cancel();
                return Ok(false);
            }
            match self.handle.read_bulk(EP_INFO_IN, &mut buf, POLL_SLICE) {
                Ok(n) if n > 0 => {
                    if buf[..n].contains(&0xaa) {
                        return Ok(true);
                    }
                    if buf[..n].contains(&0xff) {
                        return Ok(false);
                    }
                }
                Ok(_) => {}
                Err(rusb::Error::Timeout) => {
                    if cancel.load(Ordering::SeqCst) || Instant::now() >= deadline {
                        let _ = self.bridge_cancel();
                        return Ok(false);
                    }
                }
                Err(e) => return Err(e.into()),
            }
        }
    }

    /// Capture a background calibration frame (`calibration_start` + `capture_raw_u16`).
    pub fn capture_background(&self, width: u16, height: u16) -> Result<Vec<u16>> {
        self.calibration_start()?;
        self.capture_raw_u16(width, height)
    }

    /// Capture raw 16-bit sensor frame in non-enclave mode (`0x00 0x09` on `EP_CMD_OUT`,
    /// read `2 * width * height` bytes from `EP_IMG_IN`).
    pub fn capture_raw_u16(&self, width: u16, height: u16) -> Result<Vec<u16>> {
        let expected_bytes = 2usize * usize::from(width) * usize::from(height);
        self.write_cmd(&[0x00, 0x09])?;

        let mut raw = vec![0u8; expected_bytes];
        let mut received = 0usize;
        while received < expected_bytes {
            let n = self
                .handle
                .read_bulk(EP_IMG_IN, &mut raw[received..], IMG_TIMEOUT)?;
            if n == 0 {
                break;
            }
            received += n;
        }
        if received != expected_bytes {
            return Err(DriverError::ShortRead {
                expected: expected_bytes,
                actual: received,
            });
        }

        Ok(raw
            .chunks_exact(2)
            .map(|c| u16::from_le_bytes([c[0], c[1]]))
            .collect())
    }
}

pub fn decode_fw_version(buf: &[u8]) -> Result<u16> {
    if buf.len() < 2 {
        return Err(DriverError::ShortRead {
            expected: 2,
            actual: buf.len(),
        });
    }
    Ok(u16::from_be_bytes([buf[0], buf[1]]))
}

pub fn decode_dimension(buf: &[u8]) -> Result<u16> {
    if buf.is_empty() {
        return Err(DriverError::ShortRead {
            expected: 1,
            actual: 0,
        });
    }
    Ok(u16::from(buf[0]) + 1)
}

/// Subtract background frame (`sub_18003C280`: `raw[i].saturating_sub(bg[i])`),
/// normalize to 8-bit grayscale, and return `(pixels, raw_adc_diff_span)`.
pub fn subtract_background_with_stats(raw: &[u16], background: &[u16]) -> (Vec<u8>, u16) {
    let mut min = u16::MAX;
    let mut max = 0u16;
    let diff: Vec<u16> = raw
        .iter()
        .zip(background.iter())
        .map(|(&r, &b)| {
            let d = r.saturating_sub(b);
            if d < min {
                min = d;
            }
            if d > max {
                max = d;
            }
            d
        })
        .collect();
    let span = if diff.is_empty() {
        0
    } else {
        max.saturating_sub(min)
    };
    (normalize_u16_to_u8(&diff), span)
}

/// Subtract background frame (`sub_18003C280`: `raw[i].saturating_sub(bg[i])`)
/// and normalize to 8-bit grayscale.
pub fn subtract_background_u8(raw: &[u16], background: &[u16]) -> Vec<u8> {
    subtract_background_with_stats(raw, background).0
}

/// Normalize 16-bit sensor samples into an 8-bit grayscale buffer.
pub fn normalize_u16_to_u8(samples: &[u16]) -> Vec<u8> {
    let min = samples.iter().copied().min().unwrap_or(0);
    let max = samples.iter().copied().max().unwrap_or(1);
    let span = (max.saturating_sub(min)).max(1);

    samples
        .iter()
        .map(|&px| (((px.saturating_sub(min)) as u32) * 255 / (span as u32)) as u8)
        .collect()
}

/// Encode an 8-bit grayscale buffer as a binary PGM (`P5`) image.
pub fn encode_pgm(width: u16, height: u16, pixels: &[u8]) -> Vec<u8> {
    let mut out = format!("P5\n{} {}\n255\n", width, height).into_bytes();
    out.extend_from_slice(pixels);
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decodes_fw_version_big_endian() {
        assert_eq!(decode_fw_version(&[0x27, 0x20]).unwrap(), 0x2720);
    }

    #[test]
    fn decodes_dimension_plus_one() {
        assert_eq!(decode_dimension(&[0x5f, 0x00]).unwrap(), 96);
    }

    #[test]
    fn subtracts_background_and_encodes_pgm() {
        let (pixels, span) = subtract_background_with_stats(&[100, 355], &[120, 100]);
        assert_eq!(span, 255);
        assert_eq!(pixels, vec![0, 255]);
        let pgm = encode_pgm(2, 1, &pixels);
        assert!(pgm.starts_with(b"P5\n2 1\n255\n"));
        assert_eq!(&pgm[pgm.len() - 2..], &[0, 255]);
    }
}
