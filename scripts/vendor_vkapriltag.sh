#!/usr/bin/env bash
# Re-vendors the GPU AprilTag detector (vkapriltag), the patched AprilTag 3 core it decodes with,
# and the Vulkan headers into photon-targeting/src/main/native/thirdparty, then regenerates the
# embedded SPIR-V shaders and the symbol-prefix header.
#
# Usage: scripts/vendor_vkapriltag.sh <path to a vkapriltag checkout that has been built with CMake>
#
# Needs: glslangValidator, cmake, nm, python3, and Vulkan headers in /usr/include (libvulkan-dev).
set -euo pipefail

SRC="${1:?path to built vkapriltag checkout}"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
TP="$ROOT/photon-targeting/src/main/native/thirdparty"
LIB="$SRC/apriltags_vulkan/library"
AT="$SRC/build/_deps/apriltag-src"   # FetchContent copy, already patched by vkapriltag's build

[ -d "$AT" ] || { echo "Build vkapriltag with CMake first so the patched apriltag exists" >&2; exit 1; }

# --- vkapriltag -----------------------------------------------------------------------------------
VK="$TP/vkapriltag"
rm -rf "$VK" && mkdir -p "$VK/src/vk" "$VK/src/gpu" "$VK/src/common" "$VK/include" "$VK/shaders"
cp "$SRC/LICENSE" "$SRC/NOTICE" "$VK/"
cp -r "$LIB/include/vkapriltag" "$VK/include/"
rm -f "$VK/include/vkapriltag/FramePipeline.h" "$VK/include/vkapriltag/PoseEstimator.h" \
      "$VK/include/vkapriltag/apriltag_family.h" "$VK/include/vkapriltag/common/pgm_io.h"
for f in vk/Context.cpp vk/Buffer.cpp vk/Shader.cpp vk/ComputePipeline.cpp vk/PipelineCache.cpp \
         vk/QueryPool.cpp gpu/GpuDetector.cpp gpu/QuadDecode.cpp TagDecoder.cpp RefineEdges.cpp \
         common/WorkerPool.cpp common/Profiler.cpp; do
  cp "$LIB/src/$f" "$VK/src/$f"
done
# Only the shaders vkapriltag's own build compiles (its SHADER_SOURCES list), plus shared includes
python3 - "$LIB/CMakeLists.txt" > "$VK/shaders/SHADERS.txt" <<'PY'
import re, sys
block = re.search(r"set\(SHADER_SOURCES(.*?)\)", open(sys.argv[1]).read(), re.S).group(1)
print("\n".join(s.split("/")[-1] for s in block.split()))
PY
while read -r s; do cp "$LIB/shaders/$s" "$VK/shaders/"; done < "$VK/shaders/SHADERS.txt"
cp "$LIB"/shaders/*.glsl "$VK/shaders/"
git -C "$SRC" rev-parse HEAD > "$VK/VERSION"

# --- AprilTag 3 core (patched), only what the decoder needs ---------------------------------------
ATV="$TP/apriltag3"
rm -rf "$ATV" && mkdir -p "$ATV/include/common" "$ATV/src/common"
cp "$AT/LICENSE.md" "$ATV/"
for f in apriltag.h apriltag_math.h tag36h11.h tag16h5.h; do cp "$AT/$f" "$ATV/include/"; done
for f in apriltag.c apriltag_quad_thresh.c tag36h11.c tag16h5.c; do cp "$AT/$f" "$ATV/src/"; done
cp "$AT"/common/*.h "$ATV/include/common/"
for f in "$AT"/common/*.c; do
  case "$(basename "$f")" in getopt.c) ;; *) cp "$f" "$ATV/src/common/" ;; esac
done
git -C "$AT" describe --tags > "$ATV/VERSION" 2>/dev/null || echo "v3.4.5" > "$ATV/VERSION"

# --- Vulkan headers -------------------------------------------------------------------------------
VH="$TP/vulkan-headers"
rm -rf "$VH" && mkdir -p "$VH/include/vulkan" "$VH/include/vk_video"
cp /usr/include/vulkan/vulkan.h /usr/include/vulkan/vulkan_core.h /usr/include/vulkan/vk_platform.h \
   "$VH/include/vulkan/"
cp /usr/include/vk_video/*.h "$VH/include/vk_video/"
grep -m1 "VK_HEADER_VERSION " "$VH/include/vulkan/vulkan_core.h" > "$VH/VERSION"

# --- Embedded SPIR-V --------------------------------------------------------------------------------
python3 "$ROOT/scripts/gpu_apriltag_codegen.py" shaders "$VK/shaders" \
  "$ROOT/photon-targeting/src/main/native/gpu/cpp/embedded_shaders.cpp"

# --- Symbol prefixing (needs a compile, see the codegen script) ------------------------------------
python3 "$ROOT/scripts/gpu_apriltag_codegen.py" symbols "$ROOT"
echo "Done. Review the diff, then rebuild photon-targeting."
