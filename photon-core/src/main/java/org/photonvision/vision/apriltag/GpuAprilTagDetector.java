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

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.photonvision.common.logging.LogGroup;
import org.photonvision.common.logging.Logger;
import org.photonvision.jni.GpuAprilTagJNI;
import org.wpilib.vision.apriltag.AprilTagDetection;
import org.wpilib.vision.apriltag.AprilTagDetector;

/**
 * AprilTag detection with the image-processing stages (decimation, thresholding, connected
 * components, boundary clustering) on the GPU via Vulkan compute, and quad fitting plus tag
 * decoding on the CPU. Results use WPILib's {@link AprilTagDetection} so the rest of the pipeline
 * is unchanged.
 *
 * <p>Software (CPU-emulated) Vulkan devices are refused unless {@code APRILTAG_VK_ALLOW_CPU=1} is
 * set, since they are far slower than the CPU detector.
 */
public class GpuAprilTagDetector implements AutoCloseable {
    private static final Logger logger = new Logger(GpuAprilTagDetector.class, LogGroup.General);

    /**
     * Result of probing for a usable GPU.
     *
     * @param available True if detection can run on the GPU
     * @param software True if the device is a software (CPU) Vulkan implementation
     * @param description Device description, or why the GPU path is unavailable
     */
    public record DeviceStatus(boolean available, boolean software, String description) {}

    /** Per-frame timings of the last detection, in milliseconds. */
    public record Timings(double gpuMs, double quadFitMs, double decodeMs, double totalMs) {}

    private static DeviceStatus cachedStatus = null;
    private static boolean cachedAllowSoftware = false;

    /**
     * Probe for a usable GPU (cached after the first call). Never throws.
     *
     * @param allowSoftware Accept a software Vulkan implementation (for tests)
     */
    public static synchronized DeviceStatus queryDevice(boolean allowSoftware) {
        if (cachedStatus != null && cachedAllowSoftware == allowSoftware) return cachedStatus;
        DeviceStatus status;
        try {
            if (!GpuAprilTagJNI.isSupportedPlatform()) {
                status = new DeviceStatus(false, false, "GPU AprilTag detection is only built for Linux");
            } else {
                var r = GpuAprilTagJNI.queryDevice(allowSoftware);
                status = new DeviceStatus(Boolean.parseBoolean(r[0]), Boolean.parseBoolean(r[1]), r[2]);
            }
        } catch (UnsatisfiedLinkError e) {
            status = new DeviceStatus(false, false, "photon-targeting native library is not loaded");
        } catch (RuntimeException e) {
            status = new DeviceStatus(false, false, e.getMessage());
        }
        if (status.available()) {
            logger.info("GPU AprilTag detection available: " + status.description());
        } else {
            logger.info("GPU AprilTag detection unavailable: " + status.description());
        }
        cachedStatus = status;
        cachedAllowSoftware = allowSoftware;
        return status;
    }

    /**
     * @return null if a frame of this size and these settings can be processed on the GPU, otherwise
     *     the reason it can't
     */
    public static String unsupportedReason(
            int width, int height, AprilTagDetector.Config config, String family) {
        if (config.quadSigma != 0) return "blur is not supported on the GPU";
        if (config.quadDecimate != Math.rint(config.quadDecimate)) {
            return "fractional decimation is not supported on the GPU";
        }
        try {
            return GpuAprilTagJNI.unsupportedReason(
                    width, height, Math.max(1, (int) config.quadDecimate), family);
        } catch (UnsatisfiedLinkError e) {
            return "photon-targeting native library is not loaded";
        }
    }

    private long handle;
    private final String family;
    private final int width;
    private final int height;

    /**
     * Create a detector for frames of one size. Throws RuntimeException if the GPU can't be used.
     *
     * @param width Frame width in pixels
     * @param height Frame height in pixels
     * @param config Detector config (decimation, refine edges, sharpening, threads)
     * @param quadParams Quad threshold parameters
     * @param family AprilTag family name, e.g. "tag36h11"
     * @param allowSoftware Accept a software Vulkan implementation (for tests)
     */
    public GpuAprilTagDetector(
            int width,
            int height,
            AprilTagDetector.Config config,
            AprilTagDetector.QuadThresholdParameters quadParams,
            String family,
            boolean allowSoftware) {
        this.family = family;
        this.width = width;
        this.height = height;
        this.handle =
                GpuAprilTagJNI.create(
                        width,
                        height,
                        Math.max(1, (int) config.quadDecimate),
                        config.refineEdges,
                        config.decodeSharpening,
                        config.numThreads,
                        quadParams.criticalAngle,
                        quadParams.maxLineFitMSE,
                        quadParams.minWhiteBlackDiff,
                        family,
                        allowSoftware);
    }

    public boolean matches(int width, int height) {
        return this.width == width && this.height == height;
    }

    /**
     * Detect tags. The image must be 8-bit single channel and the size this detector was created for.
     */
    public AprilTagDetection[] detect(Mat gray) {
        if (handle == 0) throw new IllegalStateException("GPU AprilTag detector was closed");
        if (gray.type() != CvType.CV_8UC1) {
            throw new IllegalArgumentException("GPU AprilTag detector needs an 8-bit grayscale image");
        }
        double[] packed = GpuAprilTagJNI.detect(handle, gray.getNativeObjAddr());
        int n = packed.length / GpuAprilTagJNI.DETECTION_STRIDE;
        var out = new AprilTagDetection[n];
        for (int i = 0; i < n; i++) {
            int o = i * GpuAprilTagJNI.DETECTION_STRIDE;
            double[] homography = new double[9];
            System.arraycopy(packed, o + 3, homography, 0, 9);
            double[] corners = new double[8];
            System.arraycopy(packed, o + 14, corners, 0, 8);
            out[i] =
                    new AprilTagDetection(
                            family,
                            (int) packed[o],
                            (int) packed[o + 1],
                            (float) packed[o + 2],
                            homography,
                            packed[o + 12],
                            packed[o + 13],
                            corners);
        }
        return out;
    }

    public Timings lastTimings() {
        if (handle == 0) return new Timings(0, 0, 0, 0);
        var t = GpuAprilTagJNI.lastTimings(handle);
        return new Timings(t[0], t[1], t[2], t[3]);
    }

    @Override
    public void close() {
        if (handle != 0) {
            GpuAprilTagJNI.destroy(handle);
            handle = 0;
        }
    }
}
