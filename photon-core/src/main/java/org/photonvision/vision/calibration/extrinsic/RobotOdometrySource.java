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

import java.util.ArrayList;
import java.util.List;
import org.photonvision.common.dataflow.networktables.NetworkTablesManager;
import org.wpilib.networktables.DoubleArraySubscriber;
import org.wpilib.networktables.PubSubOption;

/** Supplies the robot's odometry-only pose, as published by PhotonLib's calibration helper. */
public interface RobotOdometrySource {
    /**
     * One odometry sample.
     *
     * @param timestampMicros Time of the sample in the coprocessor's NetworkTables time base
     * @param x Robot x, meters
     * @param y Robot y, meters
     * @param thetaRad Robot heading, radians
     */
    record Sample(long timestampMicros, double x, double y, double thetaRad) {}

    /** Returns every sample received since the previous call, oldest first. */
    List<Sample> poll();

    /**
     * Reads {@code /photonvision/.robot/odometry}, a double array of [x, y, thetaRadians] published
     * every robot loop by PhotonLib's {@code ExtrinsicCalibrationPublisher}.
     */
    final class NetworkTablesSource implements RobotOdometrySource {
        public static final String TABLE = ".robot";
        public static final String ODOMETRY_TOPIC = "odometry";

        private final DoubleArraySubscriber subscriber;

        public NetworkTablesSource() {
            subscriber =
                    NetworkTablesManager.getInstance()
                            .kRootTable
                            .getSubTable(TABLE)
                            .getDoubleArrayTopic(ODOMETRY_TOPIC)
                            .subscribe(
                                    new double[0],
                                    PubSubOption.SEND_ALL,
                                    PubSubOption.KEEP_DUPLICATES,
                                    PubSubOption.pollStorage(200));
        }

        @Override
        public List<Sample> poll() {
            var queue = subscriber.readQueue();
            List<Sample> out = new ArrayList<>(queue.length);
            for (var v : queue) {
                if (v.value == null || v.value.length < 3) continue;
                out.add(new Sample(v.timestamp, v.value[0], v.value[1], v.value[2]));
            }
            return out;
        }
    }
}
