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
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.opencv.core.Point;
import org.opencv.core.Size;
import org.photonvision.common.LoadJNI;
import org.photonvision.vision.calibration.CameraCalibrationCoefficients;
import org.photonvision.vision.calibration.CameraLensModel;
import org.photonvision.vision.calibration.JsonMatOfDouble;
import org.photonvision.vision.frame.Frame;
import org.photonvision.vision.frame.FrameStaticProperties;
import org.photonvision.vision.frame.FrameThresholdType;
import org.photonvision.vision.opencv.CVMat;
import org.photonvision.vision.pipeline.result.CVPipelineResult;
import org.photonvision.vision.target.TrackedTarget;
import org.wpilib.math.geometry.Pose2d;

/** Drives the capture state machine with simulated odometry and camera results. */
public class ExtrinsicCalibrationManagerTest {
    private static final long TICK_MICROS = 10_000;

    @BeforeAll
    public static void init() throws IOException {
        LoadJNI.loadLibraries();
    }

    private static final class FakeOdometry implements RobotOdometrySource {
        final List<Sample> pending = new ArrayList<>();

        @Override
        public List<Sample> poll() {
            var out = List.copyOf(pending);
            pending.clear();
            return out;
        }
    }

    private final long[] now = {1_000_000};
    private final FakeOdometry odometry = new FakeOdometry();
    private final List<ExtrinsicCalibrationManager.Status> broadcasts = new ArrayList<>();
    private final ExtrinsicCalibrationManager manager =
            new ExtrinsicCalibrationManager(odometry, () -> now[0], broadcasts::add);
    private final CameraCalibrationCoefficients calibration =
            new CameraCalibrationCoefficients(
                    new Size(SyntheticRobotData.WIDTH, SyntheticRobotData.HEIGHT),
                    new JsonMatOfDouble(3, 3, SyntheticRobotData.K),
                    new JsonMatOfDouble(1, 8, SyntheticRobotData.DIST),
                    new double[0],
                    List.of(),
                    new Size(8, 8),
                    0.025,
                    CameraLensModel.LENSMODEL_OPENCV,
                    null);
    private long sequence = 0;

    private CVPipelineResult result(Map<Integer, double[]> tags) {
        List<TrackedTarget> targets = new ArrayList<>();
        tags.forEach(
                (id, c) -> {
                    var t =
                            new TrackedTarget(
                                    List.of(
                                            new Point(c[0], c[1]),
                                            new Point(c[2], c[3]),
                                            new Point(c[4], c[5]),
                                            new Point(c[6], c[7])));
                    t.setFiducialId(id);
                    targets.add(t);
                });
        var fsp =
                new FrameStaticProperties(
                        SyntheticRobotData.WIDTH, SyntheticRobotData.HEIGHT, 70, calibration);
        var frame =
                new Frame(sequence, new CVMat(), new CVMat(), FrameThresholdType.NONE, now[0] * 1000, fsp);
        var r = new CVPipelineResult(sequence++, 0, 30, targets, frame);
        r.setImageCaptureTimestampNanos(now[0] * 1000);
        return r;
    }

    /** Simulate a stretch of time with the robot at {@code pose} (or moving toward it). */
    private void simulate(SyntheticRobotData data, Pose2d from, Pose2d to, double seconds) {
        int ticks = (int) (seconds * 1e6 / TICK_MICROS);
        for (int i = 0; i < ticks; i++) {
            now[0] += TICK_MICROS;
            double f = Math.min(1.0, (i + 1) / (ticks * 0.5));
            Pose2d pose = from.interpolate(to, f);
            if (i % 2 == 0) {
                odometry.pending.add(
                        new RobotOdometrySource.Sample(
                                now[0], pose.getX(), pose.getY(), pose.getRotation().getRadians()));
            }
            if (i % 3 == 0) {
                for (var cam : data.trueRobotToCamera.keySet()) {
                    manager.onPipelineResult(cam, cam, result(data.observe(pose, cam)));
                }
            }
        }
    }

    @Test
    public void capturesOneStopPerRestAndSolves() {
        var data = new SyntheticRobotData(99);
        data.addTags();
        data.addFullRoutine();

        manager.start(
                ExtrinsicCalibrationManager.CaptureSettings.defaults(
                        List.copyOf(data.trueRobotToCamera.keySet())));

        Pose2d prev = data.robotPoses.get(0);
        for (var pose : data.robotPoses) {
            // Drive for the first half, then sit still
            simulate(data, prev, pose, 2.5);
            prev = pose;
        }
        manager.stop();

        var status = manager.getStatus();
        assertEquals(data.robotPoses.size(), status.stopCount(), status.message());
        assertEquals(12, status.headingsCovered());
        for (var cam : status.cameras()) {
            assertTrue(cam.hasCalibration(), cam.uniqueName());
            assertTrue(
                    cam.stopsWithTags() > 10, cam.uniqueName() + " saw tags at " + cam.stopsWithTags());
        }
        assertTrue(!broadcasts.isEmpty());

        var settings = ExtrinsicSolverSettings.defaults().withDatum(false, data.floorTagIds, List.of());
        var result = manager.solve(settings, id -> Optional.empty());
        assertTrue(result.success(), result.message());
        for (var cam : result.cameras()) {
            var truth = data.trueRobotToCamera.get(cam.uniqueName());
            double err = cam.robotToCamera().getTranslation().getDistance(truth.getTranslation());
            assertTrue(err < 0.01, cam.uniqueName() + " translation error " + err);
        }
    }

    @Test
    public void movingDuringCollectionDiscardsTheStop() {
        var data = new SyntheticRobotData(5);
        data.addTags();
        manager.start(ExtrinsicCalibrationManager.CaptureSettings.defaults(List.of("front-left")));

        Pose2d a = new Pose2d();
        // Still just long enough to settle and start collecting, then a nudge before 5 frames arrive
        simulate(data, a, a, 0.55);
        assertEquals(ExtrinsicCalibrationManager.Phase.COLLECTING, manager.getPhase());
        Pose2d nudged = new Pose2d(0.05, 0, a.getRotation());
        simulate(data, a, nudged, 0.04);
        assertEquals(0, manager.getStatus().stopCount());
        assertEquals(ExtrinsicCalibrationManager.Phase.WAITING_FOR_STILL, manager.getPhase());
    }

    @Test
    public void noStopWithoutOdometry() {
        var data = new SyntheticRobotData(6);
        data.addTags();
        manager.start(ExtrinsicCalibrationManager.CaptureSettings.defaults(List.of("front-left")));
        for (int i = 0; i < 300; i++) {
            now[0] += TICK_MICROS;
            manager.onPipelineResult(
                    "front-left", "front-left", result(data.observe(new Pose2d(), "front-left")));
        }
        var status = manager.getStatus();
        assertEquals(0, status.stopCount());
        assertTrue(!status.odometryConnected());
    }

    @Test
    public void medianCornersRejectsOutlier() {
        var samples =
                List.of(
                        new double[] {1, 1, 1, 1, 1, 1, 1, 1},
                        new double[] {2, 2, 2, 2, 2, 2, 2, 2},
                        new double[] {100, 100, 100, 100, 100, 100, 100, 100});
        var m = ExtrinsicCalibrationManager.medianCorners(samples);
        for (double v : m) assertEquals(2, v, 1e-12);
    }
}
