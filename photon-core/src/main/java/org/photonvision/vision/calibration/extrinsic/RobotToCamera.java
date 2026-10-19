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
import org.jetbrains.annotations.Nullable;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;

/**
 * The stored transform from the robot origin (drivetrain center, on the floor) to a camera, in the
 * WPILib camera convention (X out of the lens, Y left, Z up). Distances in meters, angles in
 * degrees (extrinsic roll-pitch-yaw, same as {@link Rotation3d#Rotation3d(double, double,
 * double)}).
 *
 * @param x Robot frame X, meters
 * @param y Robot frame Y, meters
 * @param z Height above the floor, meters
 * @param rollDeg Roll, degrees
 * @param pitchDeg Pitch, degrees
 * @param yawDeg Yaw, degrees
 * @param source Where this came from: {@link #SOURCE_MANUAL} or {@link #SOURCE_CALIBRATED}
 * @param updatedAtMillis Unix time this was set
 * @param stdDevs 1-sigma uncertainty from calibration (x, y, z meters; roll, pitch, yaw degrees),
 *     or null for manual entries
 * @param rmsReprojectionErrorPx Calibration RMS reprojection error, 0 for manual entries
 */
@Json
public record RobotToCamera(
        double x,
        double y,
        double z,
        double rollDeg,
        double pitchDeg,
        double yawDeg,
        String source,
        long updatedAtMillis,
        @Nullable double[] stdDevs,
        double rmsReprojectionErrorPx) {
    public static final String SOURCE_MANUAL = "manual";
    public static final String SOURCE_CALIBRATED = "calibrated";

    public static RobotToCamera manual(
            double x, double y, double z, double rollDeg, double pitchDeg, double yawDeg) {
        return new RobotToCamera(
                x, y, z, rollDeg, pitchDeg, yawDeg, SOURCE_MANUAL, System.currentTimeMillis(), null, 0);
    }

    public static RobotToCamera fromCalibration(ExtrinsicSolveResult.CameraResult result) {
        return new RobotToCamera(
                result.x(),
                result.y(),
                result.z(),
                result.rollDeg(),
                result.pitchDeg(),
                result.yawDeg(),
                SOURCE_CALIBRATED,
                System.currentTimeMillis(),
                result.stdDevs(),
                result.rmsReprojectionErrorPx());
    }

    public Transform3d toTransform3d() {
        return new Transform3d(
                new Translation3d(x, y, z),
                new Rotation3d(Math.toRadians(rollDeg), Math.toRadians(pitchDeg), Math.toRadians(yawDeg)));
    }

    /** NetworkTables representation: [x, y, z, qw, qx, qy, qz]. */
    public double[] toNtArray() {
        var tf = toTransform3d();
        var q = tf.getRotation().getQuaternion();
        return new double[] {tf.getX(), tf.getY(), tf.getZ(), q.getW(), q.getX(), q.getY(), q.getZ()};
    }

    public boolean isValid() {
        return Double.isFinite(x)
                && Double.isFinite(y)
                && Double.isFinite(z)
                && Double.isFinite(rollDeg)
                && Double.isFinite(pitchDeg)
                && Double.isFinite(yawDeg);
    }
}
