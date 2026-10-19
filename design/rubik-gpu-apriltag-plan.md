# GPU-Accelerated AprilTag Detection on the Rubik Pi 3

Status: proposal. Nothing here is implemented yet.

## 1. Summary

**Goal:** more camera-frames per second on a Rubik Pi 3, either more FPS per camera or an extra camera,
by moving the expensive part of AprilTag detection onto the Adreno 643 GPU, which PhotonVision does not
use today.

**Answers to the open questions:**

- **Which library?** PhotonVision calls WPILib's `AprilTagDetector`. That is a thin wrapper around
  the AprilRobotics **apriltag3** C library, vendored inside WPILib's `apriltag` native artifact. PhotonVision
  has no AprilTag C/C++ of its own; see `AprilTagDetectionPipe.java:49`.
- **Do we need to fork it?** Not WPILib, and not in the "patch upstream and maintain a fork" sense. The
  new native library will **vendor a copy of apriltag3** (BSD-2-Clause) only for its CPU decode stage. It
  will add a small entry point, "decode these quads", and replace everything upstream of that with GPU
  code. This is the same approach FRC 971 uses for its CUDA detector.
- **What goes on the GPU?** Everything from the raw grayscale image to candidate quads: decimate,
  threshold, connected components, boundary extraction, clustering/sorting, line fitting and quad
  fitting. In apriltag3 profiles these stages are roughly 90% of detection time. Decoding the tag bits,
  edge refinement and pose estimation stay on the CPU. They are cheap and branchy, and scale with the
  number of tags rather than the number of pixels.
- **Which GPU API?** **Vulkan compute.** There is a working open-source starting point,
  `yojobama/vkapriltag`, a Vulkan port of 971's CUDA pipeline (Apache-2.0). It runs at 1280×800 with
  decimate 2 in about 2.1 ms GPU and 3.1 ms end to end on an Orange Pi 5 Mali-G610. Vulkan also works
  with both Qualcomm's proprietary Adreno driver and Mesa turnip. The same code could later speed up
  Orange Pi 5 (Mali) and Raspberry Pi 5 users. **OpenCL is the fallback** if Adreno's Vulkan compute
  turns out to be buggy or slow; Phase 0 decides this.

**Important finding:** some of the 20 fps-with-3-cameras problem is not the detector at all.
`decimate` defaults to 1 (full resolution), every camera's detector gets 4 threads, and the Rubik Pi
image pins PhotonVision to the 4 A78 cores (`AllowedCPUs=4-7` in `install_rubikpi3.sh`). That means 12
detector threads compete for 4 cores. On top of that, every frame is MJPEG-decoded to color and then
converted back to gray on the CPU. The plan fixes these first (Phase 1). Those fixes are cheap, help
every platform, and give the honest baseline the GPU work must beat.

## 2. How a frame is processed today

Per camera, one `VisionRunner` thread and one `StreamRunnable` thread:

| Step | Where | Cost driver |
| --- | --- | --- |
| MJPEG decode → **BGR** | cscore, CPU (libjpeg-turbo) | full-res color decode, chroma upsampling, YCbCr→BGR |
| Copy into a Java `CVMat` | `USBFrameProvider.java:75` | 1280×800×3 copy |
| BGR → gray | `GrayscalePipe.java:30` via `CpuImageProcessor` | full-res pass |
| `AprilTagDetector.detect` | apriltag3, `threads=4`, `decimate=1` | **dominant**: threshold, union-find, clustering, quad fit |
| Pose (single-tag, multi-tag PnP) | Java + OpenCV | small, scales with tags |
| Stream: resize, gray→BGR, draw, **JPEG encode** ×2 | stream thread + cscore | 2 JPEG encodes per frame |

Notes:

- The OV9281 UVC board only reaches high frame rates in **MJPEG** (YUYV 1280×800 is about 10 fps over
  USB 2), so MJPEG decode is unavoidable. It should be cheaper, though: decoding straight to grayscale
  skips chroma processing and color conversion.
- The sensor is monochrome, so color decode and the BGR round trips are pure waste.
- Published apriltag3 stage profiles put segmentation (threshold, union-find, clustering, quad fit) at
  about 90% of detection time, and decode plus refinement at about 5–10%. There is no published
  breakdown for 1280×800 on a Kryo A78; Phase 0 measures it.

## 3. Target architecture

```
            ┌────────── CPU: VisionRunner thread, per camera ───────────┐
 USB MJPEG ─► decode → GRAY ─► upload ┐                      ┌─► quad decode + refine (apriltag3,
            │ (libjpeg-turbo)  (memcpy │                      │   CPU, few threads) → detections
            │                   into    ▼                      │   → pose / multi-tag (unchanged)
            │              host-visible GPU buffer)            │
            └──────────────────────────┼──────────────────────┼──────────────────────────────────┘
                                       ▼                      │
              ┌──────── GPU: shared Vulkan device + queue ────┴──────────┐
              │ decimate + tile min/max → threshold → union-find CCL     │
              │ → boundary points → compaction → sort by blob            │
              │ → blob filter → line-fit moments → quad fit → quads      │
              └──────────────────────────────────────────────────────────┘
```

### 3.1 Native library: `photon-apriltag-gpu` (new repository)

This follows the packaging pattern of `photon-libcamera-gl-driver` and `rubik_jni`: a separate repo
that publishes `org.photonvision:photon-apriltag-gpu-java` and `photon-apriltag-gpu-jni:linuxarm64`,
plus `linuxx86-64` for CI and desktop testing.

- **`vk/`**: a Vulkan context shared by **all cameras**: one `VkInstance`/`VkDevice`, one compute
  queue, and a lock or submission thread. Each camera gets a `DetectorInstance` holding its own buffers,
  descriptor sets and command buffers, sized for its resolution, with pipelines cached per shader.
- **`shaders/`** (GLSL to SPIR-V at build time), derived from vkapriltag / 971:
  1. decimate (2×, optional) and 4×4 tile min/max;
  2. min/max dilation and binarize to black/white/unknown (the apriltag3 threshold, including
     `min_white_black_diff`);
  3. connected-component labeling (block-based union-find, path compression);
  4. boundary points between adjacent black/white components, with gradient direction;
  5. stream compaction and a radix sort keyed by (blob pair, angle);
  6. blob extents and size filters (`min_cluster_pixels`, max cluster size);
  7. line-fit moment prefix sums, error filter, corner peak picking and quad fit
     (`max_line_fit_mse`, `critical_angle`, `max_nmaxima`).

  Stage 7 can start on the CPU, as vkapriltag does, and move to the GPU once correct.
- **`cpu/`**: a vendored apriltag3 copy (BSD-2) with one added function,
  `apriltag_decode_quads(td, image, quads) -> zarray<apriltag_detection_t>`. It reuses upstream
  `quad_decode`, `refine_edges` and deduplication, so ID decoding, Hamming distance, decision margin and
  homography behave exactly as today.
- **`jni/`**: `GpuAprilTagJNI` with:
  - `createDetector(w, h, config)` and `destroy`;
  - `setConfig(...)`, which mirrors `AprilTagDetector.Config` and `QuadThresholdParameters`;
  - `addFamily(name)`;
  - `detect(long matPtr) -> AprilTagDetection[]`, using WPILib's public `AprilTagDetection`
    constructor so the rest of PhotonVision is unchanged;
  - `isAvailable()`, `deviceInfo()`, and `lastTimings()` for per-stage GPU timestamps.
- **`bench/` and `test/`**: a standalone CLI that runs an image set through the GPU and CPU (stock
  apriltag3) detectors and compares detections and per-stage timings. CI runs it on x86-64 with Mesa
  **lavapipe** (software Vulkan), which tests correctness without a GPU.

### 3.2 PhotonVision integration

| Change | Where |
| --- | --- |
| Dependency + version, gated on `linuxarm64` (and `linuxx86-64` for tests) | `build.gradle`, `photon-core/build.gradle` (same pattern as rknn, tflite, libcamera) |
| `JNITypes.GPU_APRILTAG("photonapriltaggpu")` | `common/LoadJNI.java` |
| Load on QCS6490; also attempt on any platform when the setting is forced on | `photon-server/.../Main.java` |
| `GpuAprilTagDetectionPipe` with the same params record as `AprilTagDetectionPipe` | `vision/pipe/impl/` |
| `AprilTagPipeline` picks the GPU pipe when enabled and available, and falls back to CPU (with a log and UI warning) on init failure or a runtime error | `vision/pipeline/AprilTagPipeline.java` |
| Global setting `aprilTagAcceleration`: `AUTO` (GPU when available on a supported platform), `GPU`, `CPU` | `HardwareSettings.java`, UI Device/Settings card |
| GPU status (device, driver, Vulkan version, working/not) in the existing `gpuAcceleration` string | `UIPhotonConfiguration.java:61` (already has a "TODO add support for other types of GPU accel") |
| Per-camera GPU time shown next to processing time | `CVPipelineResult`, `UIDataPublisher`, dashboard |

ArUco detection (`ArucoPipeline`) is out of scope.

### 3.3 Things that stay identical

- Detection filtering (`decisionMargin`, `hammingDist`), single-tag pose, multi-tag PnP, NT output
  and the stream all stay the same.
- `refineEdges` stays on the CPU, using the full-resolution gray image. Decimate 2 on the GPU
  therefore keeps sub-pixel corner accuracy, just as on the CPU.

## 4. Phases

### Phase 0: Measure and de-risk (on a Rubik Pi 3, before writing production code)

1. **Baseline profile** with 1, 2, 3 and 4 OV9281 cameras at 1280×800 MJPEG, current defaults:
   - per-camera FPS, latency, and CPU per core;
   - time split between MJPEG decode, gray conversion, `detect` (with apriltag3's internal time profile
     via `debug`), pose, and stream encode.
   - Repeat with decimate 2 and with threads 1 and 2.
   - Add an AprilTag case to `BenchmarkTest.java`, which only has reflective cases today.
2. **Driver check** on the PhotonVision Rubik image (Ubuntu 26.04 after `install_rubikpi3.sh`):
   - `vulkaninfo` and `clinfo`;
   - which `qcom-adreno*` packages are installed (`qcom-adreno1` is; `-vulkan1` and `-cl1` are
     unknown);
   - Adreno subgroup size and supported subgroup ops, `shaderInt64`, 64-bit atomics, max workgroup
     size, and dmabuf import support.
3. **Spike:** build `vkapriltag` on the Rubik Pi and run it on PhotonVision's test images
   (`test-resources/testimages/apriltag*`) and recorded OV9281 frames:
   - check that its detections match apriltag3;
   - measure GPU ms at decimate 1 and 2, and with 3 concurrent instances.
   - If Vulkan is unavailable or badly broken, repeat the threshold and union-find stages as an
     OpenCL micro-port to compare.

**Exit criteria:**
- GPU segmentation takes ≤ 5 ms per 1280×800 frame (decimate 2) with 3 concurrent cameras;
- detections match the CPU on the test set;
- the profile confirms that detection, not MJPEG decode, is the dominant CPU cost after Phase 1.

If these aren't met, stop after Phase 1.

### Phase 1: CPU quick wins (no GPU, helps every platform)

1. **Decode straight to grayscale** for AprilTag/ArUco pipelines. Ask cscore for a GRAY sink (cscore
   can decode MJPEG to gray; verify), or decode with libjpeg-turbo `JCS_GRAYSCALE`. This removes chroma
   work, the BGR→gray pass and two thirds of the copy. The input stream would show gray, which is fine
   for a mono sensor.
2. **Thread budgeting:** today the default is `threads = 4` per camera regardless of camera count. Give
   detector threads a global budget, split across active cameras by default, with the user value as an
   override.
3. **Decimation guidance:** surface "decimate 2" as the recommended setting for 1280×800 in the UI and
   docs (refine edges keeps corners accurate), with a range trade-off note. Do not silently change
   existing pipelines' defaults.
4. **Stream cost:** throttle MJPEG stream encoding (FPS cap, lower resolution by default). Consider
   letting stream threads run on the A55 cores, since the image currently allows CPUs 4–7 only; this
   needs a change in photon-image-modifier.
5. Re-measure. This is the baseline Phase 2+ must beat.

### Phase 2: GPU detector library (separate repo)

1. Bring up the Vulkan context, buffer management and SPIR-V build, on desktop first.
2. Port the vkapriltag / 971 stages, matching apriltag3 parameters exactly. Keep quad fit on the CPU
   at first.
3. Add the vendored apriltag3 `apriltag_decode_quads` entry point.
4. Add the parity test harness and CLI benchmark; set up CI on lavapipe.
5. Run on the Rubik Pi: tune workgroup sizes for Adreno (wave size 64/128), move quad fit to the GPU,
   and add per-stage timestamp queries.
6. Make it multi-camera safe: shared device and queue, per-camera instances, thread-safe submission.

### Phase 3: PhotonVision integration (this repo)

Everything in section 3.2:
- the setting, automatic fallback and status UI;
- loading the GPU library on QCS6490;
- tests: the pipe returns detections identical to `AprilTagDetectionPipe` on test images, using
  lavapipe in CI;
- docs: a "GPU acceleration on Rubik Pi 3" page and the benchmark page;
- photon-image-modifier: make sure the Vulkan (or OpenCL) ICD packages are installed on the Rubik
  image.

### Phase 4: Throughput tuning

1. **Pipelining:** submit frame N to the GPU while the CPU decodes frame N+1 and does pose for N−1.
   This uses double-buffered device buffers per camera.
2. **Zero-copy:** decode MJPEG directly into a mapped, host-visible GPU buffer (unified memory). This
   removes the upload memcpy. Use dmabuf/EGLImage import if the driver supports it.
3. **Scheduling across cameras:** batch submissions, one queue, fair ordering.
4. Re-run the Phase 0 matrix (1–4 cameras) and publish the results in the docs benchmarks section.

### Phase 5: Optional extensions

- An OpenCL backend, if the Adreno proprietary OpenCL turns out faster than Vulkan.
- Enabling the Vulkan path on Orange Pi 5 (Mali-G610) and Raspberry Pi 5 (V3D) after validation.
- Hexagon DSP for MJPEG-related work, only if decode is the bottleneck. FastCV covers only blur and
  threshold-type operations, and DSP access on Ubuntu is unconfirmed.

## 5. Expected gains (hypotheses for Phase 0 to confirm)

- **GPU side:** vkapriltag reports about 2 ms of GPU time per 1280×800 frame (decimate 2) on a
  Mali-G610. If the Adreno 643 is in the same class, one GPU could handle on the order of 150–300
  camera-frames/s, so the GPU is unlikely to be the limit for 3–4 cameras.
- **CPU side per frame after offload:** grayscale MJPEG decode, the upload memcpy, quad decode
  (≈1 ms), pose, and stream encode. If detection is about 80–90% of today's per-frame CPU time, the same
  4 cores could process several times as many frames. Camera MJPEG rate (OV9281 tops out around 100–120
  fps) and USB bandwidth would then become the limits.
- **Order of impact, roughly:** Phase 1 decimate/threads, then Phase 3 GPU offload, then Phase 1
  gray decode, then Phase 4 pipelining. This ordering is a guess until Phase 0 numbers exist.

## 6. Correctness requirements

- **Same tags found:** on the regression image set, the GPU path finds the same tag IDs as stock
  apriltag3 with the same settings (allowing ±1% detection-rate difference on hard images), with
  Hamming distance and decision margin within small tolerances.
- **Corners:** after `refineEdges`, corners within 0.1 px of the CPU path on the test set.
- **No silent degradation:** any GPU error, timeout or device loss falls back to the CPU detector for
  that camera and shows a UI warning.
- **Deterministic tests:** union-find and sorting on the GPU can reorder blobs. The final detection
  list is sorted the same way as the CPU path so tests are deterministic.

## 7. Risks

| Risk | Mitigation |
| --- | --- |
| Adreno proprietary Vulkan compute bugs or missing features (subgroups, int64 atomics used by sort keys) | Phase 0 `vulkaninfo` checks; 32-bit key packing; OpenCL fallback backend; Mesa turnip as an alternative driver |
| GPU driver packages missing or different on the PhotonVision Rubik image (26.04 dist-upgrade) | Pin and install them in photon-image-modifier; `isAvailable()` plus automatic CPU fallback |
| GPU results differ subtly from apriltag3 (971's detector is not bit-identical) | Parity harness with tolerances in CI and on device; decode/refine reuse upstream code |
| MJPEG decode becomes the new bottleneck | Phase 1 gray decode; Phase 4 decode into the GPU buffer; measure before investing further |
| Maintenance burden of a separate native repo | Same CI/publishing pattern as existing JNI repos; lavapipe-based CI; keep the shader count small |
| Thermal throttling with CPU and GPU both loaded | Measure sustained 10-minute runs in Phase 0/4; report temperatures via `SystemMonitorQCS6490` |
| vkapriltag is young (early stage, few users) | Treat it as a reference, not a dependency; port into our repo with attribution (Apache-2.0) and own it |

## 8. Open questions for the team

1. Which camera resolution, decimation and FPS do you actually run, and do you need the color input
   stream at all (mono sensor)?
2. Is changing photon-image-modifier (installing Vulkan/OpenCL packages, CPU affinity) acceptable as
   part of this work?
3. Should GPU acceleration be `AUTO` (on by default when it works) or opt-in for the first season?

## 9. References

- PhotonVision Rubik Pi 3 image: `.github/workflows/build.yml` (`rubikpi3`, `LINUX_QCS6490`);
  photon-image-modifier `install_rubikpi3.sh`.
- vkapriltag (Vulkan, Apache-2.0): <https://github.com/yojobama/vkapriltag>
- FRC 971 CUDA AprilTag: <https://github.com/frc971/971-Robot-Code/tree/master/frc971/orin>
- Team 766 apriltags_cuda: <https://github.com/Team766/apriltags_cuda>
- Wang & Olson, *AprilTag 2* (IROS 2016): <https://april.eecs.umich.edu/pdfs/wang2016iros.pdf>
- Qualcomm Linux graphics (Adreno: GLES 3.2, OpenCL 3.0, Vulkan 1.1):
  <https://docs.qualcomm.com/doc/80-70022-19/topic/graphics-overview.html>
- Qualcomm Adreno packaging: <https://github.com/qualcomm-linux/pkg-adreno>
- Mesa freedreno device table (FD643 is an A635-family speed bin):
  <https://gitlab.freedesktop.org/mesa/mesa/-/raw/main/src/freedreno/common/freedreno_devices.py>
- Rubik Pi OpenCL on the Ubuntu image (community):
  <https://community.rubikpi.ai/t/how-can-i-enable-opencl-on-rubik-pi-when-the-ubuntu-image-lacks-libopencl-so/385>
- OpenCV FastCV HAL: <https://docs.opencv.org/4.x/d5/d06/tutorial_building_fastcv.html>
