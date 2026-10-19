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

import org.photonvision.common.dataflow.DataChangeService;
import org.photonvision.common.dataflow.events.OutgoingUIEvent;

/** Pushes extrinsic calibration status to the web UI. */
final class ExtrinsicCalibrationBroadcaster {
    private ExtrinsicCalibrationBroadcaster() {}

    static void broadcast(ExtrinsicCalibrationManager.Status status) {
        DataChangeService.getInstance()
                .publishEvent(OutgoingUIEvent.wrappedOf("extrinsicCalibration", status));
    }
}
