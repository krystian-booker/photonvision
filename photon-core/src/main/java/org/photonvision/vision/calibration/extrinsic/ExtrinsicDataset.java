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

package org.photonvision.vision.calibration.extrinsic;

import io.avaje.jsonb.Json;
import java.util.List;

/**
 * Everything recorded during a robot extrinsic calibration session. Self-contained so that it can
 * be exported, re-solved later with different settings, attached to bug reports, or used as a test
 * fixture.
 *
 * @param version Format version of this dataset
 * @param tagSizeMeters Edge length of the black square of the AprilTags used
 * @param cameras Intrinsics of every camera that contributed observations
 * @param stops Each time the robot came to rest, the odometry pose and the tags each camera saw
 */
@Json
public record ExtrinsicDataset(
        int version, double tagSizeMeters, List<CameraInfo> cameras, List<Stop> stops) {
    public static final int CURRENT_VERSION = 1;

    /**
     * Intrinsics of one camera at the resolution used during capture.
     *
     * @param uniqueName PhotonVision camera unique name
     * @param nickname User facing camera name
     * @param width Image width in pixels
     * @param height Image height in pixels
     * @param cameraMatrix Row-major 3x3 OpenCV camera matrix
     * @param distCoeffs OpenCV distortion coefficients (k1, k2, p1, p2, k3, k4, k5, k6)
     */
    @Json
    public record CameraInfo(
            String uniqueName,
            String nickname,
            int width,
            int height,
            double[] cameraMatrix,
            double[] distCoeffs) {}

    /**
     * The robot at rest.
     *
     * @param timestampSeconds Coprocessor time at which this stop was recorded
     * @param odomX Odometry-only (not vision fused) robot x, meters
     * @param odomY Odometry-only robot y, meters
     * @param odomThetaRad Odometry-only robot heading, radians
     * @param observations Tags seen by each camera while at rest
     */
    @Json
    public record Stop(
            double timestampSeconds,
            double odomX,
            double odomY,
            double odomThetaRad,
            List<TagObservation> observations) {}

    /**
     * One AprilTag seen by one camera at one stop.
     *
     * @param camera Unique name of the camera
     * @param tagId Fiducial ID
     * @param corners Raw (distorted) pixel corners x0,y0..x3,y3 in PhotonVision detection order
     * @param frames Number of frames that were combined into these corners
     */
    @Json
    public record TagObservation(String camera, int tagId, double[] corners, int frames) {}
}
