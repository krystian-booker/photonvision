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

package org.photonvision.vision.pipe.impl;

import java.util.List;
import org.photonvision.common.hardware.Platform;
import org.photonvision.common.logging.LogGroup;
import org.photonvision.common.logging.Logger;
import org.photonvision.vision.apriltag.AprilTagAcceleration;
import org.photonvision.vision.apriltag.AprilTagFamily;
import org.photonvision.vision.apriltag.GpuAprilTagDetector;
import org.photonvision.vision.opencv.CVMat;
import org.photonvision.vision.pipe.CVPipe;
import org.wpilib.vision.apriltag.AprilTagDetection;
import org.wpilib.vision.apriltag.AprilTagDetector;

/**
 * Detects AprilTags on the CPU (WPILib's AprilTagDetector) or, when enabled and possible, on the
 * GPU ({@link GpuAprilTagDetector}). Any GPU problem falls back to the CPU detector.
 */
public class AprilTagDetectionPipe
        extends CVPipe<
                CVMat, List<AprilTagDetection>, AprilTagDetectionPipe.AprilTagDetectionPipeParams> {
    private static final Logger logger =
            new Logger(AprilTagDetectionPipe.class, LogGroup.VisionModule);

    /** Which detector produced the last result. */
    public enum Backend {
        CPU,
        GPU
    }

    private AprilTagDetector m_detector = new AprilTagDetector();

    private GpuAprilTagDetector m_gpuDetector = null;
    // Set when the GPU can't be used with the current params; cleared when params change
    private String m_gpuUnavailableReason = null;
    private Backend m_lastBackend = Backend.CPU;
    private GpuAprilTagDetector.Timings m_lastGpuTimings = null;

    public AprilTagDetectionPipe() {
        super();

        m_detector.addFamily("tag16h5");
        m_detector.addFamily("tag36h11");
    }

    /**
     * Whether the GPU should be attempted for this acceleration mode on this machine.
     *
     * @param mode The configured mode
     */
    public static boolean gpuRequested(AprilTagAcceleration mode) {
        return switch (mode == null ? AprilTagAcceleration.AUTO : mode) {
            case CPU -> false;
            case GPU -> true;
            case AUTO -> Platform.isQCS6490();
        };
    }

    @Override
    protected List<AprilTagDetection> process(CVMat in) {
        if (in.getMat().empty()) {
            return List.of();
        }

        if (m_detector == null) {
            throw new RuntimeException("Apriltag detector was released!");
        }

        if (m_gpuUnavailableReason == null && gpuRequested(params.acceleration())) {
            var gpuResult = tryGpu(in);
            if (gpuResult != null) return gpuResult;
        }

        m_lastBackend = Backend.CPU;
        var ret = m_detector.detect(in.getMat());

        if (ret == null) {
            return List.of();
        }

        return List.of(ret);
    }

    /** Returns null (after recording why) if the GPU couldn't produce a result. */
    private List<AprilTagDetection> tryGpu(CVMat in) {
        var mat = in.getMat();
        try {
            if (m_gpuDetector == null || !m_gpuDetector.matches(mat.cols(), mat.rows())) {
                closeGpu();
                var status = GpuAprilTagDetector.queryDevice(false);
                if (!status.available()) {
                    disableGpu(status.description());
                    return null;
                }
                var reason =
                        GpuAprilTagDetector.unsupportedReason(
                                mat.cols(), mat.rows(), params.detectorParams(), params.family().getNativeName());
                if (reason != null) {
                    disableGpu(reason);
                    return null;
                }
                m_gpuDetector =
                        new GpuAprilTagDetector(
                                mat.cols(),
                                mat.rows(),
                                params.detectorParams(),
                                params.quadParams(),
                                params.family().getNativeName(),
                                false);
                logger.info("Using the GPU for AprilTag detection at " + mat.cols() + "x" + mat.rows());
            }
            var ret = m_gpuDetector.detect(mat);
            m_lastGpuTimings = m_gpuDetector.lastTimings();
            m_lastBackend = Backend.GPU;
            return List.of(ret);
        } catch (RuntimeException | UnsatisfiedLinkError e) {
            closeGpu();
            disableGpu(e.getMessage());
            return null;
        }
    }

    private void disableGpu(String reason) {
        m_gpuUnavailableReason = reason == null ? "unknown error" : reason;
        logger.warn("AprilTag detection is using the CPU: " + m_gpuUnavailableReason);
    }

    private void closeGpu() {
        if (m_gpuDetector != null) {
            m_gpuDetector.close();
            m_gpuDetector = null;
        }
    }

    public Backend getLastBackend() {
        return m_lastBackend;
    }

    /** Null unless the last frame ran on the GPU. */
    public GpuAprilTagDetector.Timings getLastGpuTimings() {
        return m_lastBackend == Backend.GPU ? m_lastGpuTimings : null;
    }

    /** Why the GPU isn't being used, or null if it is (or hasn't been tried yet). */
    public String getGpuUnavailableReason() {
        return m_gpuUnavailableReason;
    }

    @Override
    public void setParams(AprilTagDetectionPipeParams newParams) {
        if (this.params == null || !this.params.equals(newParams)) {
            m_detector.setConfig(newParams.detectorParams());
            m_detector.setQuadThresholdParameters(newParams.quadParams());

            m_detector.clearFamilies();
            m_detector.addFamily(newParams.family().getNativeName());

            // GPU detectors bake their params in; rebuild on the next frame and give the GPU another
            // chance (e.g. after blur was turned back off)
            closeGpu();
            m_gpuUnavailableReason = null;
        }

        super.setParams(newParams);
    }

    @Override
    public void release() {
        closeGpu();
        m_detector.close();
        m_detector = null;
    }

    public static record AprilTagDetectionPipeParams(
            AprilTagFamily family,
            AprilTagDetector.Config detectorParams,
            AprilTagDetector.QuadThresholdParameters quadParams,
            AprilTagAcceleration acceleration) {
        /** CPU-only params (used by tests and callers that don't care about acceleration). */
        public AprilTagDetectionPipeParams(
                AprilTagFamily family,
                AprilTagDetector.Config detectorParams,
                AprilTagDetector.QuadThresholdParameters quadParams) {
            this(family, detectorParams, quadParams, AprilTagAcceleration.CPU);
        }
    }
}
