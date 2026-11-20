#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BIN_SRC="$SCRIPT_DIR/target/release/elan-fprintd"

if [[ ! -x "$BIN_SRC" ]]; then
  echo "Release binary not found at $BIN_SRC. Run 'cargo build --release' first." >&2
  exit 1
fi

install -m 755 "$BIN_SRC" /usr/local/bin/elan-fprintd
install -m 644 "$SCRIPT_DIR/99-elan-0c4b.rules" /etc/udev/rules.d/99-elan-0c4b.rules
udevadm control --reload-rules
udevadm trigger --subsystem-match=usb

mkdir -p /etc/systemd/system/fprintd.service.d
cat > /etc/systemd/system/fprintd.service.d/override.conf <<'EOF'
[Service]
ExecStart=
ExecStart=/usr/local/bin/elan-fprintd
EOF

systemctl daemon-reload
systemctl restart fprintd.service
echo "elan-fprintd installed and fprintd.service restarted."
