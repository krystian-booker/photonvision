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

package org.photonvision.common.configuration;

import io.avaje.jsonb.Json;
import java.util.Objects;
import org.photonvision.vision.apriltag.AprilTagAcceleration;

@Json
public class HardwareSettings {
    public int ledBrightnessPercentage = 100;

    /** Where AprilTag detection runs. AUTO uses the GPU on validated platforms (Rubik Pi 3). */
    public AprilTagAcceleration aprilTagAcceleration = AprilTagAcceleration.AUTO;

    @Override
    public int hashCode() {
        final int prime = 31;
        int result = 1;
        result = prime * result + ledBrightnessPercentage;
        result = prime * result + Objects.hashCode(aprilTagAcceleration);
        return result;
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj) return true;
        if (obj == null) return false;
        if (getClass() != obj.getClass()) return false;
        HardwareSettings other = (HardwareSettings) obj;
        if (ledBrightnessPercentage != other.ledBrightnessPercentage) return false;
        if (aprilTagAcceleration != other.aprilTagAcceleration) return false;
        return true;
    }

    @Override
    public String toString() {
        return "HardwareSettings [ledBrightnessPercentage="
                + ledBrightnessPercentage
                + ", aprilTagAcceleration="
                + aprilTagAcceleration
                + "]";
    }
}
