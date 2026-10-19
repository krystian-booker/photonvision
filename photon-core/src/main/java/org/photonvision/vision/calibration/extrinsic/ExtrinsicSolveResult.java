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
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;

/**
 * Output of {@link ExtrinsicSolver}. Angles are reported in degrees and distances in meters so the
 * UI can display them directly.
 *
 * @param success True if the solver produced a usable result
 * @param message Human readable summary or failure reason
 * @param cameras Per-camera robot-to-camera transforms
 * @param tags Final tag poses (estimated in self-survey mode, as given otherwise)
 * @param odometryScale Estimated odometry scale factor (1 = odometry is metrically correct)
 * @param rmsReprojectionErrorPx RMS corner reprojection error over all observations
 * @param iterations Optimizer iterations used
 * @param stopsUsed Number of robot stops in the dataset
 * @param warnings Problems worth showing to the user
 */
@Json
public record ExtrinsicSolveResult(
        boolean success,
        String message,
        List<CameraResult> cameras,
        List<TagResult> tags,
        double odometryScale,
        double rmsReprojectionErrorPx,
        int iterations,
        int stopsUsed,
        List<String> warnings) {
    public static ExtrinsicSolveResult failure(String message) {
        return new ExtrinsicSolveResult(
                false, message, List.of(), List.of(), 1.0, 0, 0, 0, List.of(message));
    }

    /**
     * Robot-to-camera transform of one camera. The camera frame follows the usual WPILib convention
     * (X out of the lens, Y left, Z up), matching what {@code PhotonPoseEstimator} expects.
     *
     * @param uniqueName Camera unique name
     * @param nickname Camera nickname
     * @param x Robot frame X, meters
     * @param y Robot frame Y, meters
     * @param z Height above the floor, meters
     * @param rollDeg Roll, degrees
     * @param pitchDeg Pitch, degrees
     * @param yawDeg Yaw, degrees
     * @param stdDevs 1-sigma uncertainty: x, y, z (meters) then rotation about robot x, y, z
     *     (degrees)
     * @param rmsReprojectionErrorPx RMS reprojection error of this camera's corners
     * @param observations Number of tag observations used
     * @param stops Number of stops at which this camera saw at least one tag
     * @param warnings Camera specific warnings
     */
    @Json
    public record CameraResult(
            String uniqueName,
            String nickname,
            double x,
            double y,
            double z,
            double rollDeg,
            double pitchDeg,
            double yawDeg,
            double[] stdDevs,
            double rmsReprojectionErrorPx,
            int observations,
            int stops,
            List<String> warnings) {
        public Pose3d robotToCamera() {
            return new Pose3d(
                    new Translation3d(x, y, z),
                    new Rotation3d(
                            Math.toRadians(rollDeg), Math.toRadians(pitchDeg), Math.toRadians(yawDeg)));
        }
    }

    /**
     * Pose of one tag in the calibration world frame.
     *
     * @param id Fiducial ID
     * @param x X, meters
     * @param y Y, meters
     * @param z Z, meters
     * @param rollDeg Roll, degrees
     * @param pitchDeg Pitch, degrees
     * @param yawDeg Yaw, degrees
     * @param floorTag True if this tag was constrained to lie on the floor
     * @param fixed True if this tag came from a known layout
     * @param observations Number of times this tag was observed
     */
    @Json
    public record TagResult(
            int id,
            double x,
            double y,
            double z,
            double rollDeg,
            double pitchDeg,
            double yawDeg,
            boolean floorTag,
            boolean fixed,
            int observations) {}
}
