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

package org.photonvision.jni;

/**
 * Native bindings for the Vulkan GPU AprilTag detector (vkapriltag) in the photon-targeting JNI
 * library. Linux only; on other platforms {@link #isSupportedPlatform()} is false and nothing else
 * should be called. Prefer the higher-level wrapper in photon-core over calling these directly.
 */
public class GpuAprilTagJNI {
    /** Number of doubles describing one detection in the array returned by {@link #detect}. */
    public static final int DETECTION_STRIDE = 22;

    /**
     * @return true if this build of the native library includes the GPU detector (Linux)
     */
    public static native boolean isSupportedPlatform();

    /**
     * Probe for a usable Vulkan device, creating the shared device context on success.
     *
     * @param allowSoftware Accept a software (CPU) Vulkan implementation such as lavapipe
     * @return {available ("true"/"false"), software ("true"/"false"), description or failure reason}
     */
    public static native String[] queryDevice(boolean allowSoftware);

    /**
     * @return null if the configuration can run on the GPU, otherwise the reason it can't
     */
    public static native String unsupportedReason(
            int width, int height, int decimation, String family);

    /**
     * Create a detector for one camera.
     *
     * @return native handle; throws RuntimeException with the reason on failure
     */
    public static native long create(
            int width,
            int height,
            int decimation,
            boolean refineEdges,
            double decodeSharpening,
            int numThreads,
            double criticalAngleRad,
            double maxLineFitMSE,
            int minWhiteBlackDiff,
            String family,
            boolean allowSoftware);

    public static native void destroy(long handle);

    /**
     * Detect tags in an 8-bit single-channel cv::Mat of the configured size.
     *
     * @param handle Detector handle
     * @param matAddr {@code Mat.getNativeObjAddr()}
     * @return {@link #DETECTION_STRIDE} doubles per detection: id, hamming, decision margin,
     *     homography (9, row-major), center x, center y, corners x0 y0 .. x3 y3
     */
    public static native double[] detect(long handle, long matAddr);

    /**
     * @return {gpu ms, quad fit ms, decode ms, total ms} for the last {@link #detect} call
     */
    public static native double[] lastTimings(long handle);
}
