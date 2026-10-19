#!/usr/bin/env bash
#
# Copyright (C) Photon Vision.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
#
# AprilTag CPU vs GPU benchmark for a PhotonVision coprocessor (written for the Rubik Pi 3).
#
# 1. Records system and GPU driver information.
# 2. Stops PhotonVision (restarted on exit) and records frames from every MJPEG camera.
# 3. Replays the frames through PhotonVision's AprilTag detection with 1..N simulated cameras in
#    several configurations (today's defaults, CPU tuned, GPU) using the same CPU cores the
#    PhotonVision service is allowed to use.
# 4. Packs everything into one .tar.gz to share.
#
# Usage: sudo ./rubik_apriltag_benchmark.sh [options]
#   --seconds N        seconds per benchmark run (default 15)
#   --frames N         frames to record per camera (default 300)
#   --cameras LIST     simulated camera counts (default 1,2,3,4)
#   --scenarios LIST   default baseline,cpu-d2,gpu-d1,gpu-d2 (see --help of the jar)
#   --frames-dir DIR   skip recording and use existing .jpg frames
#   --resolution WxH   capture resolution (default 1280x800)
#   --fps N            capture frame rate requested from the cameras (default 100)
#   --jar PATH         PhotonVision jar (default: taken from the photonvision service)
set -uo pipefail

SECONDS_PER_RUN=15
FRAMES=300
CAMERAS="1,2,3,4"
SCENARIOS="baseline,cpu-d2,gpu-d1,gpu-d2"
FRAMES_DIR=""
RESOLUTION="1280x800"
CAPTURE_FPS=100
JAR=""

while [ $# -gt 0 ]; do
  case "$1" in
    --seconds) SECONDS_PER_RUN="$2"; shift 2 ;;
    --frames) FRAMES="$2"; shift 2 ;;
    --cameras) CAMERAS="$2"; shift 2 ;;
    --scenarios) SCENARIOS="$2"; shift 2 ;;
    --frames-dir) FRAMES_DIR="$2"; shift 2 ;;
    --resolution) RESOLUTION="$2"; shift 2 ;;
    --fps) CAPTURE_FPS="$2"; shift 2 ;;
    --jar) JAR="$2"; shift 2 ;;
    -h|--help) sed -n '18,36p' "$0"; exit 0 ;;
    *) echo "Unknown option $1" >&2; exit 1 ;;
  esac
done

if [ "$(id -u)" -ne 0 ]; then
  echo "Run with sudo (needed to stop/start PhotonVision and read camera devices)." >&2
  exit 1
fi

OUT="${SUDO_USER:+/home/$SUDO_USER}/photonvision-benchmark-$(date +%Y%m%d-%H%M%S)"
[ -n "${SUDO_USER:-}" ] || OUT="$HOME/photonvision-benchmark-$(date +%Y%m%d-%H%M%S)"
mkdir -p "$OUT"
echo "Writing results to $OUT"

section() { echo; echo "===== $1 ====="; }

# ---------------------------------------------------------------------------------------------
section "System" | tee "$OUT/system.txt"
{
  uname -a
  echo "Model: $(tr -d '\0' < /proc/device-tree/model 2>/dev/null || echo unknown)"
  echo "Cores: $(nproc --all) (usable here: $(nproc))"
  lscpu 2>/dev/null | grep -E "Model name|CPU\(s\)|MHz" || true
  echo "Service CPU limits:"
  systemctl show photonvision -p AllowedCPUs -p CPUAffinity 2>/dev/null || true
  echo "Governors: $(cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor 2>/dev/null | sort | uniq -c | tr '\n' ' ')"
  for z in /sys/class/thermal/thermal_zone*; do
    echo "$(cat "$z/type" 2>/dev/null): $(( $(cat "$z/temp" 2>/dev/null || echo 0) / 1000 )) C"
  done
  free -m
} >> "$OUT/system.txt" 2>&1
cat "$OUT/system.txt"

# ---------------------------------------------------------------------------------------------
section "GPU drivers" | tee "$OUT/gpu.txt"
{
  echo "-- packages"
  dpkg -l 2>/dev/null | grep -i -E "adreno|vulkan|mesa|opencl|libdrm" || echo "(dpkg not available)"
  echo "-- Vulkan ICDs"
  ls -l /usr/share/vulkan/icd.d /etc/vulkan/icd.d 2>/dev/null || echo "(no ICD directories)"
  echo "-- Vulkan loader"
  ldconfig -p 2>/dev/null | grep -i libvulkan || echo "libvulkan not found"
  echo "-- vulkaninfo --summary"
  if command -v vulkaninfo > /dev/null; then vulkaninfo --summary 2>&1; else echo "vulkaninfo not installed (sudo apt install vulkan-tools)"; fi
  echo "-- clinfo"
  if command -v clinfo > /dev/null; then clinfo -l 2>&1; else echo "clinfo not installed (sudo apt install clinfo)"; fi
} >> "$OUT/gpu.txt" 2>&1
cat "$OUT/gpu.txt"

# ---------------------------------------------------------------------------------------------
section "PhotonVision"
EXEC="$(systemctl show photonvision -p ExecStart --value 2>/dev/null)"
if [ -z "$JAR" ]; then
  JAR="$(echo "$EXEC" | grep -o '[^ ;=]*\.jar' | head -1)"
fi
JAVA="$(echo "$EXEC" | grep -o 'path=[^ ;]*' | head -1 | cut -d= -f2)"
[ -x "$JAVA" ] || JAVA="$(command -v java)"
if [ ! -f "$JAR" ]; then
  echo "Couldn't find the PhotonVision jar; pass --jar /path/to/photonvision.jar" >&2
  exit 1
fi
echo "Jar: $JAR"
echo "Java: $JAVA"

# Run the benchmark on the same cores the service may use (the Rubik image allows CPUs 4-7)
ALLOWED="$(systemctl show photonvision -p AllowedCPUs --value 2>/dev/null)"
TASKSET=()
if [ -n "$ALLOWED" ] && command -v taskset > /dev/null; then
  TASKSET=(taskset -c "$ALLOWED")
  echo "Benchmark pinned to CPUs $ALLOWED, like the service"
fi

WAS_ACTIVE=0
if systemctl is-active --quiet photonvision; then
  WAS_ACTIVE=1
  echo "Stopping PhotonVision for the benchmark (it is restarted afterwards)"
  systemctl stop photonvision
fi
restart() { [ "$WAS_ACTIVE" = 1 ] && systemctl start photonvision && echo "PhotonVision restarted"; }
trap restart EXIT

# ---------------------------------------------------------------------------------------------
if [ -z "$FRAMES_DIR" ]; then
  section "Recording frames"
  if ! command -v ffmpeg > /dev/null; then
    echo "ffmpeg is needed to record frames (sudo apt install ffmpeg), or pass --frames-dir" >&2
    exit 1
  fi
  FRAMES_DIR="$OUT/frames"
  mkdir -p "$FRAMES_DIR"
  n=0
  for dev in /dev/video*; do
    # Only capture nodes that offer MJPEG
    if command -v v4l2-ctl > /dev/null; then
      v4l2-ctl -d "$dev" --list-formats 2>/dev/null | grep -q MJPG || continue
    fi
    echo "Recording $FRAMES frames from $dev at $RESOLUTION @ $CAPTURE_FPS fps"
    log="$OUT/record_$(basename "$dev").log"
    if ffmpeg -hide_banner -nostdin -loglevel info -f v4l2 -input_format mjpeg \
         -video_size "$RESOLUTION" -framerate "$CAPTURE_FPS" -i "$dev" \
         -frames:v "$FRAMES" -c:v copy "$FRAMES_DIR/cam${n}_%05d.jpg" > "$log" 2>&1; then
      echo "  camera fps reported by ffmpeg: $(grep -o 'fps=[ 0-9.]*' "$log" | tail -1)"
      n=$((n + 1))
    else
      echo "  (skipped: $(tail -1 "$log"))"
      rm -f "$FRAMES_DIR"/cam${n}_*.jpg
    fi
  done
  if [ "$n" = 0 ]; then
    echo "No MJPEG camera could be recorded" >&2
    exit 1
  fi
fi

# ---------------------------------------------------------------------------------------------
section "Benchmark" | tee "$OUT/benchmark.txt"
"${TASKSET[@]}" "$JAVA" -jar "$JAR" --benchmark-apriltag \
  --benchmark-images "$FRAMES_DIR" \
  --benchmark-cameras "$CAMERAS" \
  --benchmark-scenarios "$SCENARIOS" \
  --benchmark-seconds "$SECONDS_PER_RUN" \
  --benchmark-out "$OUT/report.json" 2>&1 | grep -v -E "\] \[(INFO|DEBUG|TRACE)\]" | tee -a "$OUT/benchmark.txt"

section "Temperatures after" | tee -a "$OUT/system.txt"
for z in /sys/class/thermal/thermal_zone*; do
  echo "$(cat "$z/type" 2>/dev/null): $(( $(cat "$z/temp" 2>/dev/null || echo 0) / 1000 )) C"
done | tee -a "$OUT/system.txt"

# Keep the share small: drop the recorded frames from the archive unless asked otherwise
tar --exclude="frames" -czf "$OUT.tar.gz" -C "$(dirname "$OUT")" "$(basename "$OUT")"
[ -n "${SUDO_USER:-}" ] && chown -R "$SUDO_USER" "$OUT" "$OUT.tar.gz"
echo
echo "Done. Share $OUT.tar.gz (frames are kept in $FRAMES_DIR)."
