#!/usr/bin/env bash
# camera_stack_diagnostics.sh
#
# Read-only diagnostics for a USB/V4L2 + GStreamer + OpenCV camera stack.
# It does NOT start streaming and does NOT change device settings.
#
# Usage:
#   bash camera_stack_diagnostics.sh
#   DEV=/dev/video0 bash camera_stack_diagnostics.sh
#   DEV_GLOB="/dev/video*" bash camera_stack_diagnostics.sh

set -Eeuo pipefail

DEV="${DEV:-}"
DEV_GLOB="${DEV_GLOB:-/dev/video*}"
TS="$(date -Is 2>/dev/null || date)"

hr() { echo "--------------------------------------------------------------------------------"; }
sec() { hr; echo "[$TS] $*"; }
have() { command -v "$1" >/dev/null 2>&1; }

echo "Camera Stack Diagnostics (read-only)"
echo "Timestamp: $TS"
echo

sec "Host / OS / Kernel / Container context"
echo "User: $(id -un)  UID:GID=$(id -u):$(id -g)"
echo "Groups: $(id -nG)"
echo "Kernel: $(uname -a)"
if [[ -r /etc/os-release ]]; then
  # shellcheck disable=SC1091
  . /etc/os-release
  echo "OS: ${PRETTY_NAME:-unknown}"
fi
if [[ -r /proc/1/cgroup ]] && grep -qiE 'docker|containerd|kubepods' /proc/1/cgroup; then
  echo "Container: likely YES (cgroup hints docker/containerd/k8s)"
else
  echo "Container: likely NO (or cannot infer)"
fi
echo

sec "V4L2 device nodes present"
ls -l $DEV_GLOB 2>/dev/null || echo "No nodes match: $DEV_GLOB"
echo

sec "Persistent camera paths (recommended for multi-camera)"
if [[ -d /dev/v4l/by-id ]]; then
  echo "/dev/v4l/by-id:"
  ls -l /dev/v4l/by-id/ || true
else
  echo "Missing: /dev/v4l/by-id"
fi
echo
if [[ -d /dev/v4l/by-path ]]; then
  echo "/dev/v4l/by-path:"
  ls -l /dev/v4l/by-path/ || true
else
  echo "Missing: /dev/v4l/by-path"
fi
echo

sec "Pick a target device (DEV)"
if [[ -z "$DEV" ]]; then
  # pick first video node if available
  first="$(ls -1 $DEV_GLOB 2>/dev/null | head -n 1 || true)"
  if [[ -n "$first" ]]; then
    DEV="$first"
    echo "DEV not provided. Using first match: DEV=$DEV"
  else
    echo "DEV not provided and no /dev/video* found. Set DEV=/dev/videoX and rerun."
  fi
else
  echo "Using DEV=$DEV"
fi
echo

sec "Who is using the device? (device busy check)"
if [[ -n "$DEV" && -e "$DEV" ]]; then
  if have fuser; then
    echo "fuser -v $DEV"
    fuser -v "$DEV" 2>/dev/null || echo "No process currently holds $DEV (or insufficient privileges)."
  else
    echo "Missing 'fuser'. Install 'psmisc' if you want this check."
  fi
else
  echo "Skipping: DEV not set or does not exist."
fi
echo

sec "Permissions / access hints"
if [[ -n "$DEV" && -e "$DEV" ]]; then
  echo "Node: $(ls -l "$DEV")"
  echo "Tip: if you are not root, you usually need to be in the 'video' group to access /dev/video*."
fi
echo

sec "V4L2 tooling (v4l2-ctl) and basic capability queries"
if have v4l2-ctl; then
  echo "v4l2-ctl version: $(v4l2-ctl --version 2>/dev/null || true)"
  echo
  echo "v4l2-ctl --list-devices"
  v4l2-ctl --list-devices 2>/dev/null || true
  echo
  if [[ -n "$DEV" && -e "$DEV" ]]; then
    echo "v4l2-ctl -d $DEV --all (read-only query)"
    v4l2-ctl -d "$DEV" --all 2>/dev/null || true
    echo
    echo "v4l2-ctl -d $DEV --list-formats-ext"
    v4l2-ctl -d "$DEV" --list-formats-ext 2>/dev/null || true
    echo
    echo "v4l2-ctl -d $DEV --list-ctrls (camera exposure/gain/etc controls)"
    v4l2-ctl -d "$DEV" --list-ctrls 2>/dev/null || true
  fi
else
  echo "Missing 'v4l2-ctl' (v4l-utils). Install it to inspect formats/controls."
fi
echo

sec "USB inventory (which device is which)"
if have lsusb; then
  echo "lsusb:"
  lsusb || true
  echo
  echo "lsusb -t (topology):"
  lsusb -t || true
else
  echo "Missing 'lsusb'. Install 'usbutils' for this check."
fi
echo

sec "Sysfs mapping: /dev/videoX -> USB path"
if [[ -n "$DEV" && -e "$DEV" ]]; then
  base="$(basename "$DEV")"
  sys="/sys/class/video4linux/$base"
  if [[ -e "$sys/device" ]]; then
    echo "Sysfs: $sys"
    echo "Resolved device path:"
    readlink -f "$sys/device" || true
    echo
    echo "Udev attributes (ID_VENDOR/ID_MODEL/ID_SERIAL if available):"
    if have udevadm; then
      udevadm info --query=all --name="$DEV" 2>/dev/null | egrep 'ID_VENDOR=|ID_MODEL=|ID_SERIAL=|ID_VENDOR_ID=|ID_MODEL_ID=|DEVPATH=' || true
    else
      echo "Missing 'udevadm' (usually in systemd/udev base)."
    fi
  else
    echo "Cannot map sysfs for $DEV (missing $sys/device)."
  fi
else
  echo "Skipping: DEV not set or does not exist."
fi
echo

sec "GStreamer presence + key plugins for USB cameras"
if have gst-launch-1.0 || have gst-inspect-1.0; then
  if have gst-inspect-1.0; then
    echo "gst-inspect-1.0 --version:"
    gst-inspect-1.0 --version 2>/dev/null || true
    echo
    for p in v4l2src videoconvert jpegdec avdec_mjpeg appsink; do
      if gst-inspect-1.0 "$p" >/dev/null 2>&1; then
        echo "OK plugin: $p"
      else
        echo "MISSING plugin: $p"
      fi
    done
    echo
    echo "If v4l2src/videoconvert/appsink are missing, install gstreamer base/good plugins + tools."
  else
    echo "gst-inspect-1.0 not found (but gst-launch-1.0 exists). Install gstreamer1.0-tools to get gst-inspect."
  fi
else
  echo "GStreamer tools not found (gst-launch-1.0 / gst-inspect-1.0)."
  echo "If OpenCV is built with GStreamer backend, missing tools/plugins often correlates with 'unable to start pipeline'."
fi
echo

sec "OpenCV presence (optional) + VideoIO environment knobs (non-invasive)"
# Try pkg-config first (system installs), then python fallback
if have pkg-config && pkg-config --exists opencv4; then
  echo "OpenCV (pkg-config opencv4): $(pkg-config --modversion opencv4 2>/dev/null || true)"
elif have python3; then
  echo "OpenCV (python3):"
  python3 - <<'PY' 2>/dev/null || true
import cv2
print(cv2.__version__)
try:
  print("Build info videoio backends snippet:")
  bi = cv2.getBuildInformation()
  for line in bi.splitlines():
    if "Video I/O:" in line or "GStreamer:" in line or "V4L/V4L2:" in line:
      print(line)
except Exception as e:
  print("Cannot read build info:", e)
PY
else
  echo "OpenCV not detectable via pkg-config or python3."
fi
echo
echo "Current env vars that influence OpenCV VideoIO:"
for k in OPENCV_VIDEOIO_DEBUG OPENCV_LOG_LEVEL OPENCV_VIDEOIO_PRIORITY_LIST OPENCV_VIDEOIO_PRIORITY_GSTREAMER OPENCV_VIDEOIO_PRIORITY_V4L2; do
  if [[ -n "${!k:-}" ]]; then
    echo "  $k=${!k}"
  fi
done
echo
echo "Notes:"
echo "  - OPENCV_VIDEOIO_PRIORITY_LIST can force backend order, e.g. 'V4L2' first."
echo "  - OPENCV_VIDEOIO_DEBUG / OPENCV_LOG_LEVEL can add verbose logging."
echo

sec "Kernel logs for camera/USB errors (read-only, last 200 lines filtered)"
if have dmesg; then
  # This does not change state; it just reads kernel ring buffer.
  dmesg | tail -n 200 | egrep -i "uvc|video|v4l2|usb|urb|stall|reset|timeout|tegra|xusb" || echo "No obvious camera/USB errors in last 200 dmesg lines."
else
  echo "Missing 'dmesg' command (unusual)."
fi
echo

sec "Diagnostic summary (heuristics)"
busy="unknown"
if [[ -n "$DEV" && -e "$DEV" && $(have fuser; echo $?) -eq 0 ]]; then
  if fuser "$DEV" >/dev/null 2>&1; then busy="yes"; else busy="no"; fi
fi

echo "Heuristics:"
echo "  - Device busy: $busy"
echo "  - GStreamer tools present: $(have gst-inspect-1.0 && echo yes || echo no)"
echo "  - v4l2-ctl present: $(have v4l2-ctl && echo yes || echo no)"
echo
echo "If you see OpenCV/GStreamer errors like 'unable to start pipeline' or 'pipeline have not been created':"
echo "  1) First ensure the device is not busy (see fuser output)."
echo "  2) Ensure GStreamer core + plugins exist (v4l2src, videoconvert, appsink)."
echo "  3) Prefer stable device paths /dev/v4l/by-id or /dev/v4l/by-path for multi-camera setups."
echo "  4) If OpenCV chooses GStreamer by default and you suspect GStreamer issues, force V4L2 via OPENCV_VIDEOIO_PRIORITY_LIST=V4L2."

hr
echo "Done."
