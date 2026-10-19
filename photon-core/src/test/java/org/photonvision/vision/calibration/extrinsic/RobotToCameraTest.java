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

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;

import io.avaje.jsonb.Jsonb;
import org.junit.jupiter.api.Test;
import org.photonvision.common.configuration.CameraConfiguration;
import org.photonvision.vision.camera.PVCameraInfo;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;

public class RobotToCameraTest {
    @Test
    public void cameraConfigurationRoundTrip() {
        var config =
                new CameraConfiguration("cam", PVCameraInfo.fromFileInfo("cam", "/dev/video99"));
        var type = Jsonb.instance().type(CameraConfiguration.class);

        // Older configs have no robotToCamera at all
        assertNull(type.fromJson(type.toJson(config)).robotToCamera);

        config.robotToCamera = RobotToCamera.manual(0.3, -0.25, 0.4, 1.0, -15.0, 30.0);
        var parsed = type.fromJson(type.toJson(config));
        assertEquals(config.robotToCamera.x(), parsed.robotToCamera.x());
        assertEquals(config.robotToCamera.pitchDeg(), parsed.robotToCamera.pitchDeg());
        assertEquals(RobotToCamera.SOURCE_MANUAL, parsed.robotToCamera.source());
    }

    @Test
    public void networkTablesArrayMatchesTransform() {
        var r2c = RobotToCamera.manual(0.3, -0.25, 0.4, 1.0, -15.0, 30.0);
        var expected =
                new Transform3d(
                        new Translation3d(0.3, -0.25, 0.4),
                        new Rotation3d(Math.toRadians(1.0), Math.toRadians(-15.0), Math.toRadians(30.0)));
        var q = expected.getRotation().getQuaternion();
        assertArrayEquals(
                new double[] {0.3, -0.25, 0.4, q.getW(), q.getX(), q.getY(), q.getZ()},
                r2c.toNtArray(),
                1e-12);
        assertFalse(RobotToCamera.manual(Double.NaN, 0, 0, 0, 0, 0).isValid());
    }
}
