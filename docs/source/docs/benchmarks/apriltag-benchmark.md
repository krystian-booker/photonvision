# AprilTag CPU vs GPU Benchmark

PhotonVision includes a benchmark that replays frames recorded from your own cameras through AprilTag detection, simulating 1 to N cameras at once. It compares:

- `baseline`: the previous defaults (CPU, decimate 1, 4 threads per camera, color decode);
- `cpu-d2`: tuned CPU (decimate 2, automatic threads, grayscale decode);
- `gpu-d1` and `gpu-d2`: the GPU detector at decimate 1 and 2.

It also checks that the CPU and GPU detectors find the same tags on your frames.

## Running it on a coprocessor

The script records frames from every MJPEG camera, runs the benchmark on the same CPU cores the PhotonVision service uses, and packs the results into a `.tar.gz`. PhotonVision is stopped while it runs and restarted afterwards.

```bash
sudo apt install ffmpeg v4l-utils vulkan-tools
sudo ./rubik_apriltag_benchmark.sh
```

Copy `scripts/rubik_apriltag_benchmark.sh` from the PhotonVision repository to the coprocessor first.

Useful options:

| Option | Default | Meaning |
| --- | --- | --- |
| `--seconds N` | 15 | Seconds per run |
| `--frames N` | 300 | Frames recorded per camera |
| `--cameras LIST` | 1,2,3,4 | Simulated camera counts |
| `--scenarios LIST` | baseline,cpu-d2,gpu-d1,gpu-d2 | What to compare (also `cpu-d1`, or custom `cpu\|gpu:d<decimate>:t<threads>:gray\|color`) |
| `--frames-dir DIR` | | Reuse previously recorded frames |
| `--resolution WxH` / `--fps N` | 1280x800 / 100 | Capture mode |

Point the cameras at a few AprilTags while recording, at a range of distances, so the frames look like real use.

## Running it directly

The benchmark is built into the PhotonVision jar:

```bash
java -jar photonvision.jar --benchmark-apriltag --benchmark-images ./frames \
  --benchmark-cameras 1,2,3 --benchmark-seconds 15 --benchmark-out report.json
```

## Reading the results

| Column | Meaning |
| --- | --- |
| `ran on` | The detector that actually ran. The reason is printed if the GPU fell back to the CPU. |
| `total fps` | Frames processed per second across all simulated cameras. |
| `fps/camera` | Frames per second each camera would get, if the camera could deliver frames that fast. |
| `decode ms` | JPEG decode time per frame. This is what MJPEG decoding costs a real camera. |
| `detect ms` | Detection time per frame, as measured by the camera's thread. |
| `cpu %` | CPU used, as a percentage of the cores available to the benchmark. |

A configuration that leaves `cpu %` well below 100 at your target frame rate has room for another camera.

Real cameras also limit the frame rate. For example, the Arducam OV9281 USB delivers about 100 fps in MJPEG at 1280×800. Detection also isn't the only per-frame cost: pose estimation and the dashboard streams take some CPU too.
