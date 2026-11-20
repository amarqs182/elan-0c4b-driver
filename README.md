# elan-0c4b-driver

Native Rust userspace USB driver, biometric matcher, and `fprintd`-compatible D-Bus daemon (`net.reactivated.Fprint`) for the **Elan Microelectronics `04f3:0c4b`** fingerprint sensor (found on Lenovo ThinkPad E14 Gen 3 / ThinkBook laptops).

## Why?

Stock open-source `libfprint` treats `04f3:0c4b` as an older swipe sensor and fails with `protocol error` during enrollment/identification. By reverse-engineering the official Windows UMDF/WinUSB driver (`WbfUsbDriver.dll`), we mapped the actual USB endpoints, command protocol, and background-subtraction pipeline and reimplemented them natively in Rust.

## Hardware & Protocol Summary

- **USB ID:** `04f3:0c4b` (Elan Microelectronics Corp., tested on FW `0x2720`, `80x80` 16-bit sensor)
- **Bulk Endpoints:**
  - `0x01` (`EP_CMD_OUT`): Command write (`SetCmdRequest`)
  - `0x83` (`EP_INFO_IN`): Command/register response read (`GetCmdRequest`)
  - `0x82` (`EP_IMG_IN`): Raw 16-bit image frame read (`SendReadRequest`)
- **Mapped Commands:**
  - `0x40 0x19` — `GetBridgeFWVersion` (2-byte big-endian `u16`)
  - `0x40 0x21 0x09` — `ReadSensorWidth` (`byte[0] + 1`)
  - `0x40 0x21 0x08` — `ReadSensorHeight` (`byte[0] + 1`)
  - `0x40 0x13` — `ReadSensorStatus`
  - `0x40 0x23` — `SetBridgeToCalibrationStart` (expects `0x03`)
  - `0x40 0x24` — `GetBridgeCalibrationMean`
  - `0x40 0x3F` — `SensorWaitFingerPress`
  - `0x40 0x3E` — `SensorWaitFingerRelease` (expects `0xAA`)
  - `0x00 0x09` — Plain raw 16-bit frame capture (`2 * width * height` bytes on `0x82`)

## Architecture

- [`src/lib.rs`](src/lib.rs) — `Elan0c4b` USB driver (`rusb`), background subtraction, and PGM image encoding.
- [`src/matcher.rs`](src/matcher.rs) — Local `9x9` mean subtraction + translation-tolerant Normalized Cross-Correlation (NCC) matcher (`±16 px`, default threshold `0.45`).
- [`src/main.rs`](src/main.rs) — CLI tool (`info`, `capture`, `capture-cal`, `enroll`, `verify`).
- [`src/bin/elan-fprintd.rs`](src/bin/elan-fprintd.rs) — System D-Bus daemon implementing `net.reactivated.Fprint.Manager` and `net.reactivated.Fprint.Device` (`/net/reactivated/Fprint/Device/0`), storing templates under `/var/lib/fprint/elan/<user>/<finger>.bin`.

## Build & Test

Requirements: Rust (`cargo`/`rustc`), `libusb-1.0` development headers (`libusb`), and `pkg-config`.

```bash
cargo test
cargo build --release
```

## CLI Usage (Testing)

```bash
# Query sensor info
./target/release/elan-0c4b-driver info

# Capture background-calibrated PGM image
./target/release/elan-0c4b-driver capture-cal /tmp/finger.pgm

# Local enroll & verify
./target/release/elan-0c4b-driver enroll right-index
./target/release/elan-0c4b-driver verify right-index
```

## System Integration (`fprintd` / PAM / Omarchy)

The included [`install.sh`](install.sh) installs `/usr/local/bin/elan-fprintd`, adds [`99-elan-0c4b.rules`](99-elan-0c4b.rules), and configures a `systemd` drop-in override at `/etc/systemd/system/fprintd.service.d/override.conf` so `fprintd.service` runs `elan-fprintd`:

```bash
sudo ./install.sh
fprintd-list "$USER"
fprintd-enroll "$USER"
fprintd-verify
```

On Omarchy, you can then run:

```bash
omarchy-setup-security-fingerprint
```

### Revert / Uninstall

```bash
sudo rm -f /etc/systemd/system/fprintd.service.d/override.conf /etc/udev/rules.d/99-elan-0c4b.rules /usr/local/bin/elan-fprintd
sudo udevadm control --reload-rules
sudo systemctl daemon-reload
sudo systemctl restart fprintd.service
```

## Security Notes

- Templates are stored locally as binary 8-bit ridge frames under `/var/lib/fprint/elan/` (owned by `root` when run via `fprintd.service`).
- Non-enclave plain capture (`0x00 0x09`) is used; this project does not implement Windows SGX/VSM enclaves.
