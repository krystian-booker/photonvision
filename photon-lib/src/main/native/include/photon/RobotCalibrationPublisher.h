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
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#pragma once

#include <wpi/math/geometry/Pose2d.hpp>
#include <wpi/nt/DoubleArrayTopic.hpp>
#include <wpi/nt/NetworkTableInstance.hpp>

namespace photon {

/**
 * Publishes the robot's odometry to PhotonVision so it can calibrate where each
 * camera is mounted on the robot (robot-to-camera transforms) instead of
 * relying on CAD.
 *
 * Call Update() every robot loop with an odometry-only pose (wheel encoders +
 * gyro, no vision measurements fused in). Then start "Robot Calibration" in
 * the PhotonVision UI and drive the robot in a stop-and-go pattern: turn to
 * several headings and drive short straight legs, pausing about a second at
 * each stop.
 */
class RobotCalibrationPublisher {
 public:
  /**
   * Publish on the given NetworkTables instance.
   *
   * @param instance The NetworkTables instance to publish on
   */
  explicit RobotCalibrationPublisher(
      wpi::nt::NetworkTableInstance instance =
          wpi::nt::NetworkTableInstance::GetDefault());

  /**
   * Publish the latest odometry pose. Call this every loop.
   *
   * @param odometryPose Robot pose from odometry only (no vision fusion)
   */
  void Update(const wpi::math::Pose2d& odometryPose);

 private:
  wpi::nt::DoubleArrayPublisher odometryPublisher;
};

}  // namespace photon
