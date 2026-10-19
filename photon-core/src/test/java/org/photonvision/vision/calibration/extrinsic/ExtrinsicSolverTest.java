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

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import io.avaje.jsonb.Jsonb;
import java.io.IOException;
import java.util.List;
import org.ejml.simple.SimpleMatrix;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.photonvision.common.LoadJNI;
import org.photonvision.estimation.OpenCVHelp;
import org.photonvision.estimation.RotTrlTransform3d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import org.wpilib.math.numbers.N8;

public class ExtrinsicSolverTest {
    // With 0.3px corner noise and ~50 stops we expect millimeter-level recovery
    private static final double MAX_TRANSLATION_ERROR_M = 0.006;
    private static final double MAX_ROTATION_ERROR_DEG = 0.25;

    @BeforeAll
    public static void init() throws IOException {
        LoadJNI.loadLibraries();
    }

    private static void assertRecovered(SyntheticRobotData data, ExtrinsicSolveResult result) {
        assertTrue(result.success(), result.message());
        assertEquals(data.trueRobotToCamera.size(), result.cameras().size());
        for (var cam : result.cameras()) {
            Transform3d truth = data.trueRobotToCamera.get(cam.uniqueName());
            Pose3d est = cam.robotToCamera();
            double transErr = est.getTranslation().getDistance(truth.getTranslation());
            double rotErr = Math.toDegrees(est.getRotation().relativeTo(truth.getRotation()).getAngle());
            // getAngle() may report 2pi - angle depending on the quaternion's sign
            rotErr = Math.min(rotErr, 360 - rotErr);
            System.out.printf(
                    "%s: translation error %.2f mm, rotation error %.3f deg, sigma xyz=(%.2f, %.2f, %.2f) mm rpy=(%.3f, %.3f, %.3f) deg, rms %.3f px%n",
                    cam.uniqueName(),
                    transErr * 1000,
                    rotErr,
                    cam.stdDevs()[0] * 1000,
                    cam.stdDevs()[1] * 1000,
                    cam.stdDevs()[2] * 1000,
                    cam.stdDevs()[3],
                    cam.stdDevs()[4],
                    cam.stdDevs()[5],
                    cam.rmsReprojectionErrorPx());
            assertTrue(
                    transErr < MAX_TRANSLATION_ERROR_M, cam.uniqueName() + " translation error " + transErr);
            assertTrue(rotErr < MAX_ROTATION_ERROR_DEG, cam.uniqueName() + " rotation error " + rotErr);
            assertTrue(cam.warnings().isEmpty(), cam.uniqueName() + " warnings: " + cam.warnings());
        }
    }

    @Test
    public void selfSurveyWithFloorTagsRecoversExtrinsics() {
        var data = new SyntheticRobotData(42);
        data.addTags();
        data.addFullRoutine();
        var dataset = data.build();

        var settings = ExtrinsicSolverSettings.defaults().withDatum(false, data.floorTagIds, List.of());
        var result = ExtrinsicSolver.solve(dataset, settings, null);
        assertRecovered(data, result);
        // Odometry was 2% long; the solver should notice
        assertEquals(1 / data.odomScale, result.odometryScale(), 0.005);
        assertTrue(result.rmsReprojectionErrorPx() < 1.0);
    }

    @Test
    public void selfSurveyWithMeasuredTagHeightRecoversExtrinsics() {
        var data = new SyntheticRobotData(11);
        data.includeFloorTags = false;
        data.addTags();
        data.addFullRoutine();
        var dataset = data.build();

        var heights =
                List.of(
                        new ExtrinsicSolverSettings.TagHeight(1, data.trueTags.get(1).getZ()),
                        new ExtrinsicSolverSettings.TagHeight(7, data.trueTags.get(7).getZ()));
        var settings = ExtrinsicSolverSettings.defaults().withDatum(false, List.of(), heights);
        var result = ExtrinsicSolver.solve(dataset, settings, null);
        assertRecovered(data, result);
    }

    @Test
    public void knownLayoutRecoversExtrinsics() {
        var data = new SyntheticRobotData(7);
        data.includeFloorTags = false;
        data.addTags();
        data.addFullRoutine();
        var dataset = data.build();

        var settings = ExtrinsicSolverSettings.defaults().withDatum(true, List.of(), List.of());
        var result = ExtrinsicSolver.solve(dataset, settings, data.trueTags);
        assertRecovered(data, result);
    }

    @Test
    public void selfSurveyWithoutDatumFails() {
        var data = new SyntheticRobotData(1);
        data.includeFloorTags = false;
        data.addTags();
        data.addFullRoutine();
        var settings = ExtrinsicSolverSettings.defaults().withDatum(false, List.of(), List.of());
        var result = ExtrinsicSolver.solve(data.build(), settings, null);
        assertFalse(result.success());
        assertTrue(result.message().contains("floor"), result.message());
    }

    @Test
    public void spinningInPlaceCannotDetermineYaw() {
        var data = new SyntheticRobotData(3);
        data.addTags();
        data.addSpinOnlyRoutine();
        var settings = ExtrinsicSolverSettings.defaults().withDatum(false, data.floorTagIds, List.of());
        var result = ExtrinsicSolver.solve(data.build(), settings, null);
        // Either initialization refuses, or every camera reports an unconstrained yaw
        if (result.success()) {
            for (var cam : result.cameras()) {
                assertTrue(
                        cam.warnings().stream().anyMatch(w -> w.contains("Yaw")),
                        cam.uniqueName() + " should warn about yaw, got " + cam.warnings());
            }
        } else {
            assertTrue(result.message().contains("yaw"), result.message());
        }
    }

    @Test
    public void projectionMatchesOpenCv() {
        double[] uv = new double[2];
        // A point 2m in front of the camera, a bit left and up, in WPILib camera coordinates
        double[] p = {2.0, 0.3, 0.2};
        assertTrue(ExtrinsicSolver.project(SyntheticRobotData.K, SyntheticRobotData.DIST, p, uv));
        Matrix<N3, N3> cameraMatrix = new Matrix<>(new SimpleMatrix(3, 3, true, SyntheticRobotData.K));
        Matrix<N8, N1> distCoeffs = new Matrix<>(new SimpleMatrix(8, 1, true, SyntheticRobotData.DIST));
        var pts =
                OpenCVHelp.projectPoints(
                        cameraMatrix,
                        distCoeffs,
                        RotTrlTransform3d.makeRelativeTo(new Pose3d()),
                        List.of(new Translation3d(p[0], p[1], p[2])));
        assertEquals(pts[0].x, uv[0], 1e-3);
        assertEquals(pts[0].y, uv[1], 1e-3);
    }

    @Test
    public void rigidHelpersRoundTrip() {
        var rot = new Rotation3d(0.3, -0.7, 2.9);
        var back = Rigid3.toRotation3d(Rigid3.fromQuaternion(rot.getQuaternion()));
        assertEquals(0, Math.toDegrees(back.relativeTo(rot).getAngle()), 1e-6);
        double[] w = {0.1, -2.0, 1.1};
        double[] w2 = Rigid3.logSO3(Rigid3.expSO3(w[0], w[1], w[2]));
        for (int i = 0; i < 3; i++) assertEquals(w[i], w2[i], 1e-9);
    }

    @Test
    public void datasetJsonRoundTrip() throws IOException {
        var data = new SyntheticRobotData(5);
        data.addTags();
        data.addFullRoutine();
        var dataset = data.build();
        var type = Jsonb.instance().type(ExtrinsicDataset.class);
        var json = type.toJson(dataset);
        var parsed = type.fromJson(json);
        assertEquals(dataset.stops().size(), parsed.stops().size());
        assertEquals(dataset.cameras().get(0).uniqueName(), parsed.cameras().get(0).uniqueName());
        assertEquals(
                dataset.stops().get(3).observations().size(), parsed.stops().get(3).observations().size());
    }
}
