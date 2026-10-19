/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.photonvision;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.networktables.DoubleArrayPublisher;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.PubSubOption;

/**
 * Publishes the robot's odometry to PhotonVision so it can calibrate where each camera is mounted
 * on the robot (robot-to-camera transforms) instead of relying on CAD.
 *
 * <p>Call {@link #update(Pose2d)} every robot loop with an <b>odometry-only</b> pose (wheel
 * encoders + gyro, no vision measurements fused in). Then start "Robot Calibration" in the
 * PhotonVision UI and drive the robot in a stop-and-go pattern: turn to several headings and drive
 * short straight legs, pausing about a second at each stop.
 *
 * <pre>{@code
 * private final RobotCalibrationPublisher calibrationPublisher = new RobotCalibrationPublisher();
 *
 * public void robotPeriodic() {
 *     calibrationPublisher.update(odometry.getPoseMeters());
 * }
 * }</pre>
 */
public class RobotCalibrationPublisher implements AutoCloseable {
    /** Table (under /photonvision) and topic PhotonVision reads odometry from. */
    public static final String kTable = ".robot";

    public static final String kOdometryTopic = "odometry";

    private final DoubleArrayPublisher odometryPublisher;

    /** Publish on the default NetworkTables instance. */
    public RobotCalibrationPublisher() {
        this(NetworkTableInstance.getDefault());
    }

    /**
     * Publish on the given NetworkTables instance.
     *
     * @param instance The NetworkTables instance to publish on
     */
    public RobotCalibrationPublisher(NetworkTableInstance instance) {
        odometryPublisher =
                instance
                        .getTable("photonvision")
                        .getSubTable(kTable)
                        .getDoubleArrayTopic(kOdometryTopic)
                        .publish(PubSubOption.SEND_ALL, PubSubOption.periodic(0.01));
    }

    /**
     * Publish the latest odometry pose. Call this every loop.
     *
     * @param odometryPose Robot pose from odometry only (no vision fusion)
     */
    public void update(Pose2d odometryPose) {
        odometryPublisher.set(
                new double[] {
                    odometryPose.getX(), odometryPose.getY(), odometryPose.getRotation().getRadians()
                });
    }

    @Override
    public void close() {
        odometryPublisher.close();
    }
}
