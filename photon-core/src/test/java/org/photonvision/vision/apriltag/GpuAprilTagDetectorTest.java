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

package org.photonvision.vision.apriltag;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.stream.Stream;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.imgcodecs.Imgcodecs;
import org.photonvision.common.LoadJNI;
import org.photonvision.common.util.TestUtils;
import org.photonvision.jni.LibraryLoader;
import org.photonvision.vision.opencv.CVMat;
import org.photonvision.vision.pipe.impl.AprilTagDetectionPipe;
import org.photonvision.vision.pipe.impl.AprilTagDetectionPipe.AprilTagDetectionPipeParams;
import org.wpilib.vision.apriltag.AprilTagDetection;
import org.wpilib.vision.apriltag.AprilTagDetector;

/**
 * Compares the GPU AprilTag detector against WPILib's CPU detector on PhotonVision's test images.
 *
 * <p>Runs on any Vulkan device; in CI that is Mesa lavapipe (software Vulkan), enabled for tests
 * with APRILTAG_VK_ALLOW_CPU=1. Skipped when no Vulkan device is available.
 */
public class GpuAprilTagDetectorTest {
    // PhotonVision's default filtering (AprilTagPipelineSettings)
    private static final double MIN_DECISION_MARGIN = 35;
    private static final int MAX_HAMMING = 0;

    // The GPU path decodes with AprilTag 3.4.5's bilinear edge refinement, WPILib with nearest-pixel
    // sampling, so corners differ slightly
    private static final double MAX_MEAN_CORNER_ERROR_PX = 0.5;
    private static final double MAX_CORNER_ERROR_PX = 1.5;

    @BeforeAll
    public static void init() throws IOException {
        LoadJNI.loadLibraries();
        LibraryLoader.loadTargeting();
    }

    private static void assumeGpu() {
        var status = GpuAprilTagDetector.queryDevice(true);
        assumeTrue(status.available(), "No Vulkan device: " + status.description());
    }

    private static AprilTagDetector.Config config(int decimate) {
        var config = new AprilTagDetector.Config();
        config.numThreads = 2;
        config.refineEdges = true;
        config.quadDecimate = decimate;
        return config;
    }

    private static AprilTagDetector.QuadThresholdParameters quadParams() {
        // Same as AprilTagPipeline
        var q = new AprilTagDetector.QuadThresholdParameters();
        q.minClusterPixels = 5;
        q.maxNumMaxima = 10;
        q.criticalAngle = 45 * Math.PI / 180.0;
        q.maxLineFitMSE = 10.0f;
        q.minWhiteBlackDiff = 5;
        q.deglitch = false;
        return q;
    }

    private static List<AprilTagDetection> filtered(AprilTagDetection[] dets) {
        return Arrays.stream(dets)
                .filter(d -> d.getDecisionMargin() >= MIN_DECISION_MARGIN && d.getHamming() <= MAX_HAMMING)
                .sorted(
                        Comparator.comparingInt(AprilTagDetection::getId)
                                .thenComparingDouble(AprilTagDetection::getCenterX))
                .toList();
    }

    private static List<Path> images() throws IOException {
        var root = TestUtils.getResourcesFolderPath(false).resolve("testimages");
        List<Path> out = new ArrayList<>();
        for (var dir : List.of(root.resolve("2024"), root.resolve("apriltag"))) {
            try (Stream<Path> s = Files.list(dir)) {
                s.filter(p -> p.toString().endsWith(".jpg") || p.toString().endsWith(".png"))
                        .filter(p -> !p.getFileName().toString().contains("16h5"))
                        .sorted()
                        .forEach(out::add);
            }
        }
        return out;
    }

    /** Grayscale, cropped so decimation divides the size evenly. */
    private static Mat loadGray(Path p, int decimate) {
        Mat gray = Imgcodecs.imread(p.toString(), Imgcodecs.IMREAD_GRAYSCALE);
        int w = gray.cols() - gray.cols() % decimate;
        int h = gray.rows() - gray.rows() % decimate;
        Mat cropped = new Mat(gray, new Rect(0, 0, w, h)).clone();
        gray.release();
        return cropped;
    }

    private static void compare(int decimate) throws IOException {
        assumeGpu();
        int compared = 0;
        double sumError = 0;
        int corners = 0;
        try (var cpu = new AprilTagDetector()) {
            cpu.setConfig(config(decimate));
            cpu.setQuadThresholdParameters(quadParams());
            cpu.addFamily("tag36h11");
            for (var path : images()) {
                Mat gray = loadGray(path, decimate);
                try (var gpu =
                        new GpuAprilTagDetector(
                                gray.cols(), gray.rows(), config(decimate), quadParams(), "tag36h11", true)) {
                    var cpuDets = filtered(cpu.detect(gray));
                    var gpuDets = filtered(gpu.detect(gray));
                    assertEquals(
                            cpuDets.stream().map(AprilTagDetection::getId).toList(),
                            gpuDets.stream().map(AprilTagDetection::getId).toList(),
                            "tag IDs differ on " + path.getFileName() + " at decimation " + decimate);
                    for (int i = 0; i < cpuDets.size(); i++) {
                        for (int k = 0; k < 4; k++) {
                            double e =
                                    Math.hypot(
                                            cpuDets.get(i).getCornerX(k) - gpuDets.get(i).getCornerX(k),
                                            cpuDets.get(i).getCornerY(k) - gpuDets.get(i).getCornerY(k));
                            assertTrue(
                                    e < MAX_CORNER_ERROR_PX,
                                    path.getFileName() + " tag " + cpuDets.get(i).getId() + " corner error " + e);
                            sumError += e;
                            corners++;
                        }
                    }
                    compared += cpuDets.size();
                } finally {
                    gray.release();
                }
            }
        }
        double mean = sumError / Math.max(corners, 1);
        System.out.printf(
                "Decimation %d: %d tags matched, mean corner difference %.3f px%n",
                decimate, compared, mean);
        assertTrue(compared > 20, "too few tags compared: " + compared);
        assertTrue(mean < MAX_MEAN_CORNER_ERROR_PX, "mean corner error " + mean);
    }

    @Test
    public void matchesCpuDetectorAtDecimation1() throws IOException {
        compare(1);
    }

    @Test
    public void matchesCpuDetectorAtDecimation2() throws IOException {
        compare(2);
    }

    @Test
    public void detectionPipeUsesGpuAndFallsBack() throws IOException {
        assumeGpu();
        var image =
                TestUtils.getResourcesFolderPath(false)
                        .resolve("testimages")
                        .resolve("2024/GeneralField1.jpg");
        Mat gray = loadGray(image, 2);
        var pipe = new AprilTagDetectionPipe();
        try {
            pipe.setParams(
                    new AprilTagDetectionPipeParams(
                            AprilTagFamily.kTag36h11, config(2), quadParams(), AprilTagAcceleration.GPU));
            var dets = pipe.run(new CVMat(gray)).output;
            assertEquals(
                    AprilTagDetectionPipe.Backend.GPU, pipe.getLastBackend(), pipe.getGpuUnavailableReason());
            assertTrue(dets.size() >= 3);
            assertNotNull(pipe.getLastGpuTimings());

            // Blur isn't supported on the GPU: same pipe falls back to the CPU
            var blurred = config(2);
            blurred.quadSigma = 0.8f;
            pipe.setParams(
                    new AprilTagDetectionPipeParams(
                            AprilTagFamily.kTag36h11, blurred, quadParams(), AprilTagAcceleration.GPU));
            pipe.run(new CVMat(gray));
            assertEquals(AprilTagDetectionPipe.Backend.CPU, pipe.getLastBackend());
            assertTrue(pipe.getGpuUnavailableReason().contains("blur"));

            // CPU mode never touches the GPU
            pipe.setParams(
                    new AprilTagDetectionPipeParams(
                            AprilTagFamily.kTag36h11, config(2), quadParams(), AprilTagAcceleration.CPU));
            pipe.run(new CVMat(gray));
            assertEquals(AprilTagDetectionPipe.Backend.CPU, pipe.getLastBackend());
        } finally {
            pipe.release();
            gray.release();
        }
    }
}
