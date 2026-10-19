###############################################################################
## Copyright (C) Photon Vision.
###############################################################################
## This program is free software: you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
##
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with this program.  If not, see <https://www.gnu.org/licenses/>.
###############################################################################

import ntcore
from wpimath import Pose2d


class RobotCalibrationPublisher:
    """Publishes the robot's odometry to PhotonVision so it can calibrate where each camera is
    mounted on the robot (robot-to-camera transforms) instead of relying on CAD.

    Call :meth:`update` every robot loop with an odometry-only pose (wheel encoders + gyro, no
    vision measurements fused in). Then start "Robot Calibration" in the PhotonVision UI and drive
    the robot in a stop-and-go pattern: turn to several headings and drive short straight legs,
    pausing about a second at each stop.
    """

    TABLE = ".robot"
    ODOMETRY_TOPIC = "odometry"

    def __init__(self, instance: ntcore.NetworkTableInstance | None = None) -> None:
        if instance is None:
            instance = ntcore.NetworkTableInstance.get_default()
        self._odometryPublisher = (
            instance.get_table("photonvision")
            .get_sub_table(self.TABLE)
            .get_double_array_topic(self.ODOMETRY_TOPIC)
            .publish(ntcore.PubSubOptions(periodic=0.01, send_all=True))
        )

    def update(self, odometryPose: Pose2d) -> None:
        """Publish the latest odometry pose. Call this every loop.

        :param odometryPose: Robot pose from odometry only (no vision fusion)
        """
        self._odometryPublisher.set(
            [odometryPose.x, odometryPose.y, odometryPose.rotation().radians()]
        )

    def close(self) -> None:
        self._odometryPublisher.close()
