# GPU-Accelerated AprilTag Detection

On coprocessors with a Vulkan-capable GPU, PhotonVision can run most of the AprilTag detector on the GPU instead of the CPU. This leaves CPU time free for more cameras or higher frame rates. It is **on by default on the Rubik Pi 3** (Adreno 643 GPU). Elsewhere you can turn it on to try it.

## How it works

AprilTag detection has two halves:

- **Image processing:** shrinking the image (decimation), thresholding, finding connected regions and grouping their edges into candidate tag outlines. Most of the time is spent here, and it does the same work on every pixel, which suits a GPU.
- **Decoding:** fitting the four corners, reading the tag's bits and refining the corners. This scales with the number of candidate tags rather than pixels, so it stays on the CPU.

PhotonVision uses [vkapriltag](https://github.com/yojobama/vkapriltag), a Vulkan port of the FRC 971/766 CUDA AprilTag detector, for the GPU half. It uses the standard AprilTag 3 library for decoding. Pose estimation, MultiTag, filtering and everything sent to the robot are unchanged.

The GPU and CPU detectors find the same tags. Corner positions can differ by a fraction of a pixel (typically about 0.1–0.2 px), because the GPU path uses AprilTag 3.4.5's sub-pixel edge refinement while WPILib's detector uses an older version.

## Settings

All of these are in **Settings → Performance**:

- **AprilTag Detection**
  - *Auto* (default): uses the GPU on the Rubik Pi 3, and the CPU elsewhere.
  - *GPU*: uses any hardware Vulkan GPU. Try this on other boards; results there are untested.
  - *CPU only*: never uses the GPU.
- **GPU Status:** the GPU that was found, or why the GPU can't be used.
- **Grayscale Capture:** for AprilTag and ArUco pipelines, decodes camera frames straight to grayscale instead of decoding to color and converting. This saves CPU time on every frame, and the OV9281 is monochrome anyway. The input stream is shown in grayscale for these pipelines.
- **Max Stream FPS:** limits how often the dashboard streams are drawn and encoded. It never limits the results sent to the robot.

The AprilTag tab on the dashboard shows which detector processed the latest frame, for example `GPU (2.3 ms)`.

## When the CPU is used anyway

If the GPU can't be used, detection quietly falls back to the CPU, and the AprilTag tab shows the reason. This happens when:

- **No GPU is found:** no Vulkan driver, or only a software (CPU-emulated) Vulkan device.
- **Blur is above 0:** blur is not supported on the GPU.
- **Decimation doesn't divide the resolution evenly.**
- **The GPU errors at runtime:** detection keeps running on the CPU until the pipeline settings change.

## Tuning for throughput

With 3 or more cameras on the same board:

- **Threads:** leave **Threads** at 0 (*Auto*). Auto splits the CPU cores across the cameras instead of giving every camera 4 threads, which oversubscribes small boards.
- **Decimation:** set **Decimate** to 2 at 1280×800. With **Refine Edges** on, corners are still found on the full-resolution image, at a small cost in maximum detection range.
- **Grayscale Capture:** keep it on.

## Measuring

See {ref}`AprilTag CPU vs GPU benchmark <docs/benchmarks/apriltag-benchmark:AprilTag CPU vs GPU Benchmark>` to compare configurations on your own hardware and cameras.
