/*
 * Copyright (C) Photon Vision.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

package org.photonvision.benchmark;

import io.avaje.jsonb.Json;
import io.avaje.jsonb.Jsonb;
import java.io.IOException;
import java.lang.management.ManagementFactory;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.stream.Stream;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.Rect;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.photonvision.vision.apriltag.AprilTagAcceleration;
import org.photonvision.vision.apriltag.AprilTagFamily;
import org.photonvision.vision.apriltag.GpuAprilTagDetector;
import org.photonvision.vision.opencv.CVMat;
import org.photonvision.vision.pipe.impl.AprilTagDetectionPipe;
import org.photonvision.vision.pipe.impl.AprilTagDetectionPipe.AprilTagDetectionPipeParams;
import org.wpilib.vision.apriltag.AprilTagDetection;
import org.wpilib.vision.apriltag.AprilTagDetector;

/**
 * Replays recorded camera frames (JPEG files, as the cameras send them over USB) through AprilTag
 * detection on several simulated cameras at once, so CPU and GPU configurations can be compared on
 * the same frames. Each simulated camera decodes its frames (like cscore does for MJPEG) and runs
 * detection in its own thread, as PhotonVision's per-camera vision threads do.
 *
 * <p>Run it via {@code java -jar photonvision.jar --benchmark-apriltag --benchmark-images DIR}.
 */
public final class AprilTagBenchmark {
    private AprilTagBenchmark() {}

    /**
     * One configuration to measure.
     *
     * @param name Short label
     * @param acceleration Where detection runs
     * @param decimate Detector decimation
     * @param threads Detector threads per camera, 0 = cores / cameras
     * @param grayDecode Decode JPEGs straight to grayscale (PhotonVision's grayscale capture)
     */
    @Json
    public record Scenario(
            String name,
            AprilTagAcceleration acceleration,
            int decimate,
            int threads,
            boolean grayDecode) {}

    /** The configurations most worth comparing, named for the benchmark CLI. */
    public static final Map<String, Scenario> PRESETS =
            Map.of(
                    "baseline",
                    new Scenario("baseline", AprilTagAcceleration.CPU, 1, 4, false),
                    "cpu-d1",
                    new Scenario("cpu-d1", AprilTagAcceleration.CPU, 1, 0, true),
                    "cpu-d2",
                    new Scenario("cpu-d2", AprilTagAcceleration.CPU, 2, 0, true),
                    "gpu-d1",
                    new Scenario("gpu-d1", AprilTagAcceleration.GPU, 1, 0, true),
                    "gpu-d2",
                    new Scenario("gpu-d2", AprilTagAcceleration.GPU, 2, 0, true));

    /**
     * Parse a comma separated list of preset names, or custom entries of the form {@code
     * cpu|gpu:d<decimate>:t<threads>:gray|color}.
     */
    public static List<Scenario> parseScenarios(String spec) {
        List<Scenario> out = new ArrayList<>();
        for (String token : spec.split(",")) {
            token = token.trim().toLowerCase(Locale.ROOT);
            if (token.isEmpty()) continue;
            if (PRESETS.containsKey(token)) {
                out.add(PRESETS.get(token));
                continue;
            }
            String[] parts = token.split(":");
            if (parts.length != 4 || !parts[1].startsWith("d") || !parts[2].startsWith("t")) {
                throw new IllegalArgumentException(
                        "Unknown scenario '"
                                + token
                                + "'. Use one of "
                                + PRESETS.keySet()
                                + " or cpu|gpu:d<decimate>:t<threads>:gray|color");
            }
            out.add(
                    new Scenario(
                            token,
                            parts[0].equals("gpu") ? AprilTagAcceleration.GPU : AprilTagAcceleration.CPU,
                            Integer.parseInt(parts[1].substring(1)),
                            Integer.parseInt(parts[2].substring(1)),
                            parts[3].equals("gray")));
        }
        return out;
    }

    /**
     * Measurements for one scenario and camera count.
     *
     * @param scenario Scenario name
     * @param cameras Simulated cameras
     * @param backend Detector that actually ran ("GPU" or "CPU", with the reason if the GPU fell
     *     back)
     * @param totalFps Frames processed per second, all cameras combined
     * @param perCameraFps Frames per second for each camera
     * @param decodeMs Mean JPEG decode (and color to gray) time per frame
     * @param detectMs Mean detection time per frame
     * @param p95FrameMs 95th percentile decode + detect time per frame
     * @param cpuCoresBusy Average CPU cores used by the process during the run
     * @param cpuPercent cpuCoresBusy as a percentage of the cores available to the process
     * @param tagsPerFrame Mean number of detections per frame
     */
    @Json
    public record RunResult(
            String scenario,
            int cameras,
            String backend,
            double totalFps,
            double[] perCameraFps,
            double decodeMs,
            double detectMs,
            double p95FrameMs,
            double cpuCoresBusy,
            double cpuPercent,
            double tagsPerFrame) {}

    /**
     * CPU vs GPU agreement on the recorded frames.
     *
     * @param decimate Decimation used
     * @param frames Frames compared
     * @param framesWithSameIds Frames where both found the same tag IDs
     * @param cpuTags Tags the CPU found (after PhotonVision's default filtering)
     * @param gpuTags Tags the GPU found
     * @param meanCornerDiffPx Mean corner distance between matched tags
     * @param maxCornerDiffPx Largest corner distance between matched tags
     * @param note Why the comparison couldn't run, if it couldn't
     */
    @Json
    public record ParityResult(
            int decimate,
            int frames,
            int framesWithSameIds,
            int cpuTags,
            int gpuTags,
            double meanCornerDiffPx,
            double maxCornerDiffPx,
            String note) {}

    /**
     * Full report.
     *
     * @param cores Cores available to the process
     * @param gpu GPU probe result
     * @param frameCount Frames loaded
     * @param width Frame width
     * @param height Frame height
     * @param parity CPU vs GPU agreement
     * @param runs Measurements
     */
    @Json
    public record Report(
            int cores,
            String gpu,
            int frameCount,
            int width,
            int height,
            List<ParityResult> parity,
            List<RunResult> runs) {}

    /** Frames as JPEG bytes (decoded per use, like MJPEG from a USB camera). */
    public static final class FrameSet {
        final List<byte[]> jpegs = new ArrayList<>();
        int width;
        int height;
    }

    /**
     * Load every .jpg/.jpeg/.png in a directory. Frames of a size other than the most common one are
     * skipped; PNGs are re-encoded to JPEG so every frame costs a JPEG decode.
     */
    public static FrameSet loadFrames(Path dir, int maxFrames) throws IOException {
        List<Path> files;
        try (Stream<Path> s = Files.list(dir)) {
            files =
                    s.filter(
                                    p -> {
                                        String n = p.getFileName().toString().toLowerCase(Locale.ROOT);
                                        return n.endsWith(".jpg") || n.endsWith(".jpeg") || n.endsWith(".png");
                                    })
                            .sorted()
                            .limit(maxFrames)
                            .toList();
        }
        Map<String, List<byte[]>> bySize = new HashMap<>();
        for (Path p : files) {
            byte[] bytes = Files.readAllBytes(p);
            Mat probe = Imgcodecs.imdecode(new MatOfByte(bytes), Imgcodecs.IMREAD_GRAYSCALE);
            if (probe.empty()) continue;
            if (!p.toString().toLowerCase(Locale.ROOT).endsWith(".jpg")
                    && !p.toString().toLowerCase(Locale.ROOT).endsWith(".jpeg")) {
                var buf = new MatOfByte();
                Imgcodecs.imencode(".jpg", probe, buf);
                bytes = buf.toArray();
                buf.release();
            }
            bySize.computeIfAbsent(probe.cols() + "x" + probe.rows(), k -> new ArrayList<>()).add(bytes);
            probe.release();
        }
        var frames = new FrameSet();
        var best = bySize.entrySet().stream().max(Comparator.comparingInt(e -> e.getValue().size()));
        if (best.isEmpty()) throw new IOException("No readable images in " + dir);
        String[] wh = best.get().getKey().split("x");
        frames.width = Integer.parseInt(wh[0]);
        frames.height = Integer.parseInt(wh[1]);
        frames.jpegs.addAll(best.get().getValue());
        return frames;
    }

    /** Decode one frame to 8-bit gray, cropped so the decimation divides it. */
    static Mat decode(byte[] jpeg, boolean grayDecode, int cropW, int cropH) {
        var buf = new MatOfByte(jpeg);
        Mat gray;
        if (grayDecode) {
            gray = Imgcodecs.imdecode(buf, Imgcodecs.IMREAD_GRAYSCALE);
        } else {
            Mat bgr = Imgcodecs.imdecode(buf, Imgcodecs.IMREAD_COLOR);
            gray = new Mat();
            Imgproc.cvtColor(bgr, gray, Imgproc.COLOR_BGR2GRAY);
            bgr.release();
        }
        buf.release();
        if (gray.cols() != cropW || gray.rows() != cropH) {
            Mat cropped = new Mat(gray, new Rect(0, 0, cropW, cropH)).clone();
            gray.release();
            return cropped;
        }
        return gray;
    }

    static AprilTagDetector.Config detectorConfig(int decimate, int threads) {
        var config = new AprilTagDetector.Config();
        config.numThreads = threads;
        config.refineEdges = true;
        config.quadDecimate = decimate;
        return config;
    }

    /** Same quad parameters as AprilTagPipeline. */
    static AprilTagDetector.QuadThresholdParameters quadParams() {
        var q = new AprilTagDetector.QuadThresholdParameters();
        q.minClusterPixels = 5;
        q.maxNumMaxima = 10;
        q.criticalAngle = 45 * Math.PI / 180.0;
        q.maxLineFitMSE = 10.0f;
        q.minWhiteBlackDiff = 5;
        q.deglitch = false;
        return q;
    }

    private static int cores() {
        return Runtime.getRuntime().availableProcessors();
    }

    private static long processCpuNanos() {
        var os = ManagementFactory.getOperatingSystemMXBean();
        if (os instanceof com.sun.management.OperatingSystemMXBean sun) {
            return sun.getProcessCpuTime();
        }
        return -1;
    }

    /** Run one scenario with {@code cameras} simulated cameras for {@code seconds}. */
    public static RunResult run(FrameSet frames, Scenario scenario, int cameras, double seconds)
            throws InterruptedException {
        int threads = scenario.threads() > 0 ? scenario.threads() : Math.max(1, cores() / cameras);
        int cropW = frames.width - frames.width % scenario.decimate();
        int cropH = frames.height - frames.height % scenario.decimate();
        var params =
                new AprilTagDetectionPipeParams(
                        AprilTagFamily.kTag36h11,
                        detectorConfig(scenario.decimate(), threads),
                        quadParams(),
                        scenario.acceleration());

        var stop = new AtomicBoolean(false);
        var ready = new CountDownLatch(cameras);
        var go = new CountDownLatch(1);
        long[] counts = new long[cameras];
        double[] decodeSum = new double[cameras];
        double[] detectSum = new double[cameras];
        long[] tagSum = new long[cameras];
        List<List<Double>> frameTimes = new ArrayList<>();
        String[] backends = new String[cameras];
        List<Thread> workers = new ArrayList<>();
        for (int c = 0; c < cameras; c++) {
            frameTimes.add(new ArrayList<>());
            final int cam = c;
            var t =
                    new Thread(
                            () -> {
                                var pipe = new AprilTagDetectionPipe();
                                pipe.setParams(params);
                                try {
                                    // Warm up: the GPU detector builds its pipelines on first use
                                    Mat warm = decode(frames.jpegs.get(0), true, cropW, cropH);
                                    pipe.run(new CVMat(warm));
                                    warm.release();
                                    ready.countDown();
                                    go.await();
                                    int i = cam * 7;
                                    while (!stop.get()) {
                                        byte[] jpeg = frames.jpegs.get(i++ % frames.jpegs.size());
                                        long t0 = System.nanoTime();
                                        Mat gray = decode(jpeg, scenario.grayDecode(), cropW, cropH);
                                        long t1 = System.nanoTime();
                                        List<AprilTagDetection> dets = pipe.run(new CVMat(gray)).output;
                                        long t2 = System.nanoTime();
                                        gray.release();
                                        counts[cam]++;
                                        decodeSum[cam] += (t1 - t0) / 1e6;
                                        detectSum[cam] += (t2 - t1) / 1e6;
                                        tagSum[cam] += dets.size();
                                        frameTimes.get(cam).add((t2 - t0) / 1e6);
                                    }
                                    backends[cam] =
                                            pipe.getLastBackend() == AprilTagDetectionPipe.Backend.GPU
                                                    ? "GPU"
                                                    : pipe.getGpuUnavailableReason() == null
                                                            ? "CPU"
                                                            : "CPU (GPU unavailable: " + pipe.getGpuUnavailableReason() + ")";
                                } catch (InterruptedException e) {
                                    Thread.currentThread().interrupt();
                                } finally {
                                    pipe.release();
                                }
                            },
                            "benchmark-camera-" + c);
            workers.add(t);
            t.start();
        }
        ready.await();
        long cpu0 = processCpuNanos();
        long wall0 = System.nanoTime();
        go.countDown();
        Thread.sleep((long) (seconds * 1000));
        stop.set(true);
        for (var t : workers) t.join();
        long wall = System.nanoTime() - wall0;
        long cpu = processCpuNanos() - cpu0;

        long total = Arrays.stream(counts).sum();
        double[] perCam = new double[cameras];
        for (int c = 0; c < cameras; c++) perCam[c] = counts[c] / (wall / 1e9);
        List<Double> all = new ArrayList<>();
        frameTimes.forEach(all::addAll);
        all.sort(Double::compare);
        double p95 = all.isEmpty() ? 0 : all.get(Math.min(all.size() - 1, (int) (all.size() * 0.95)));
        double busy = cpu0 < 0 ? -1 : cpu / (double) wall;
        return new RunResult(
                scenario.name(),
                cameras,
                backends[0],
                total / (wall / 1e9),
                perCam,
                Arrays.stream(decodeSum).sum() / Math.max(total, 1),
                Arrays.stream(detectSum).sum() / Math.max(total, 1),
                p95,
                busy,
                busy < 0 ? -1 : 100.0 * busy / cores(),
                Arrays.stream(tagSum).sum() / (double) Math.max(total, 1));
    }

    /** Compare CPU and GPU detections, after PhotonVision's default filtering, on every frame. */
    public static ParityResult parity(FrameSet frames, int decimate) {
        if (!GpuAprilTagDetector.queryDevice(false).available()) {
            return new ParityResult(decimate, 0, 0, 0, 0, 0, 0, "GPU unavailable");
        }
        int w = frames.width - frames.width % decimate;
        int h = frames.height - frames.height % decimate;
        int same = 0;
        int cpuTags = 0;
        int gpuTags = 0;
        double sum = 0;
        double max = 0;
        int n = 0;
        try (var cpu = new AprilTagDetector();
                var gpu =
                        new GpuAprilTagDetector(
                                w, h, detectorConfig(decimate, 2), quadParams(), "tag36h11", false)) {
            cpu.setConfig(detectorConfig(decimate, 2));
            cpu.setQuadThresholdParameters(quadParams());
            cpu.addFamily("tag36h11");
            for (byte[] jpeg : frames.jpegs) {
                Mat gray = decode(jpeg, true, w, h);
                var c = filtered(cpu.detect(gray));
                var g = filtered(gpu.detect(gray));
                gray.release();
                cpuTags += c.size();
                gpuTags += g.size();
                if (c.stream()
                        .map(AprilTagDetection::getId)
                        .toList()
                        .equals(g.stream().map(AprilTagDetection::getId).toList())) {
                    same++;
                    for (int i = 0; i < c.size(); i++) {
                        for (int k = 0; k < 4; k++) {
                            double e =
                                    Math.hypot(
                                            c.get(i).getCornerX(k) - g.get(i).getCornerX(k),
                                            c.get(i).getCornerY(k) - g.get(i).getCornerY(k));
                            sum += e;
                            max = Math.max(max, e);
                            n++;
                        }
                    }
                }
            }
        } catch (RuntimeException e) {
            return new ParityResult(decimate, 0, 0, 0, 0, 0, 0, e.getMessage());
        }
        return new ParityResult(
                decimate, frames.jpegs.size(), same, cpuTags, gpuTags, n == 0 ? 0 : sum / n, max, "");
    }

    private static List<AprilTagDetection> filtered(AprilTagDetection[] dets) {
        return Arrays.stream(dets)
                .filter(d -> d.getDecisionMargin() >= 35 && d.getHamming() == 0)
                .sorted(
                        Comparator.comparingInt(AprilTagDetection::getId)
                                .thenComparingDouble(AprilTagDetection::getCenterX))
                .toList();
    }

    /**
     * Run the full benchmark and print a table.
     *
     * @param imagesDir Directory of recorded frames
     * @param cameraCounts Camera counts to simulate
     * @param scenarios Scenarios to run
     * @param seconds Seconds per run
     * @param out JSON report path, or null
     * @return the report
     */
    public static Report runAll(
            Path imagesDir,
            List<Integer> cameraCounts,
            List<Scenario> scenarios,
            double seconds,
            Path out)
            throws IOException, InterruptedException {
        var frames = loadFrames(imagesDir, 2000);
        var gpu = GpuAprilTagDetector.queryDevice(false);
        System.out.printf(
                "Loaded %d frames of %dx%d from %s; %d cores available%nGPU: %s%n%n",
                frames.jpegs.size(), frames.width, frames.height, imagesDir, cores(), gpu.description());

        List<ParityResult> parity = new ArrayList<>();
        if (scenarios.stream().anyMatch(s -> s.acceleration() != AprilTagAcceleration.CPU)) {
            for (int d : scenarios.stream().map(Scenario::decimate).distinct().toList()) {
                var p = parity(frames, d);
                parity.add(p);
                System.out.printf(
                        "CPU vs GPU, decimate %d: %d/%d frames same tag IDs, tags cpu=%d gpu=%d, corner diff mean %.3f px max %.3f px %s%n",
                        p.decimate(),
                        p.framesWithSameIds(),
                        p.frames(),
                        p.cpuTags(),
                        p.gpuTags(),
                        p.meanCornerDiffPx(),
                        p.maxCornerDiffPx(),
                        p.note());
            }
            System.out.println();
        }

        System.out.printf(
                "%-10s %4s %-6s %9s %10s %9s %9s %9s %9s %6s%n",
                "scenario",
                "cams",
                "ran on",
                "total fps",
                "fps/camera",
                "decode ms",
                "detect ms",
                "p95 ms",
                "cpu %",
                "tags");
        List<RunResult> runs = new ArrayList<>();
        for (var scenario : scenarios) {
            for (int cams : cameraCounts) {
                var r = run(frames, scenario, cams, seconds);
                runs.add(r);
                System.out.printf(
                        "%-10s %4d %-6s %9.1f %10.1f %9.2f %9.2f %9.2f %9.0f %6.1f%n",
                        r.scenario(),
                        r.cameras(),
                        r.backend().startsWith("GPU") ? "GPU" : "CPU",
                        r.totalFps(),
                        r.totalFps() / r.cameras(),
                        r.decodeMs(),
                        r.detectMs(),
                        r.p95FrameMs(),
                        r.cpuPercent(),
                        r.tagsPerFrame());
                if (r.backend().startsWith("CPU (")) System.out.println("           " + r.backend());
            }
        }
        var report =
                new Report(
                        cores(),
                        gpu.description(),
                        frames.jpegs.size(),
                        frames.width,
                        frames.height,
                        parity,
                        runs);
        if (out != null) {
            Files.writeString(out, Jsonb.instance().type(Report.class).toJson(report));
            System.out.println("\nWrote " + out);
        }
        return report;
    }
}
