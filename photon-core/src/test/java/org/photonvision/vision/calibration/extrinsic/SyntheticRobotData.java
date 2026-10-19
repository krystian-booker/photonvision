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
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;
import org.ejml.simple.SimpleMatrix;
import org.photonvision.estimation.OpenCVHelp;
import org.photonvision.estimation.RotTrlTransform3d;
import org.photonvision.estimation.TargetModel;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import org.wpilib.math.numbers.N8;

/**
 * Generates a synthetic {@link ExtrinsicDataset}: a robot with known camera mounts driving a
 * stop-and-go routine among AprilTags, with noisy odometry and noisy corner detections. Projection
 * uses OpenCV via {@link OpenCVHelp#projectPoints} so the solver's own projection is checked
 * against an independent implementation.
 */
class SyntheticRobotData {
    static final double TAG_SIZE = 0.1651;
    static final int WIDTH = 1280;
    static final int HEIGHT = 800;
    static final double[] K = {900, 0, 640, 0, 905, 400, 0, 0, 1};
    static final double[] DIST = {0.05, -0.12, 0.001, -0.0005, 0.03, 0, 0, 0};

    final Map<String, Transform3d> trueRobotToCamera = new LinkedHashMap<>();
    final Map<Integer, Pose3d> trueTags = new LinkedHashMap<>();
    final List<Integer> floorTagIds = new ArrayList<>();
    final List<Pose2d> robotPoses = new ArrayList<>();

    double pixelNoise = 0.3;
    double odomTransNoise = 0.003;
    double odomTransNoisePerMeter = 0.01;
    double odomRotNoise = Math.toRadians(0.15);
    double odomScale = 1.02;
    boolean includeFloorTags = true;

    private final Random random;

    SyntheticRobotData(long seed) {
        random = new Random(seed);
        trueRobotToCamera.put(
                "front-left",
                new Transform3d(
                        new Translation3d(0.31, 0.27, 0.42),
                        new Rotation3d(0, Math.toRadians(-8), Math.toRadians(25))));
        trueRobotToCamera.put(
                "back-right",
                new Transform3d(
                        new Translation3d(-0.29, -0.24, 0.38),
                        // Pitched down so it can see the floor tags
                        new Rotation3d(Math.toRadians(1.5), Math.toRadians(18), Math.toRadians(200))));
        trueRobotToCamera.put(
                "left-side",
                new Transform3d(
                        new Translation3d(0.02, 0.33, 0.55),
                        new Rotation3d(Math.toRadians(-2), Math.toRadians(-12), Math.toRadians(92))));
    }

    /** 12 wall tags in a ring at varied heights, plus 4 floor tags. */
    void addTags() {
        int id = 1;
        for (int i = 0; i < 12; i++) {
            double a = i * Math.PI * 2 / 12;
            double r = 2.6 + 0.3 * Math.sin(3 * a);
            double z = 0.35 + 0.25 * (i % 4);
            // Face inward (tag +X points toward the origin), with some tilt
            trueTags.put(
                    id++,
                    new Pose3d(
                            new Translation3d(r * Math.cos(a), r * Math.sin(a), z),
                            new Rotation3d(0, Math.toRadians(-10 + 5 * (i % 3)), a + Math.PI)));
        }
        if (includeFloorTags) {
            for (int i = 0; i < 4; i++) {
                double a = Math.PI / 4 + i * Math.PI / 2;
                double r = 1.3;
                int fid = 20 + i;
                // Lying flat, face up: tag +X points along world +Z
                trueTags.put(
                        fid,
                        new Pose3d(
                                new Translation3d(r * Math.cos(a), r * Math.sin(a), 0.0),
                                new Rotation3d(0, -Math.PI / 2, 0).rotateBy(new Rotation3d(0, 0, a + 0.3))));
                floorTagIds.add(fid);
            }
        }
    }

    /**
     * At several spots, turn to 8 headings. After each heading, drive 0.35 m forward or sideways
     * (alternating) and stop again: the rotations fix camera x/y, the straight legs fix camera yaw.
     */
    void addFullRoutine() {
        double[][] spots = {{0, 0}, {0.6, 0.25}, {-0.5, -0.4}, {0.1, 0.7}};
        for (var spot : spots) {
            for (int h = 0; h < 8; h++) {
                var pose = new Pose2d(spot[0], spot[1], Rotation2d.fromDegrees(h * 45 + 10 * spot[0]));
                robotPoses.add(pose);
                var leg =
                        h % 2 == 0
                                ? new Transform2d(0.35, 0, new Rotation2d())
                                : new Transform2d(0, 0.35, new Rotation2d());
                robotPoses.add(pose.plus(leg));
            }
        }
    }

    /** Spin in place at one spot only: camera yaw is unobservable. */
    void addSpinOnlyRoutine() {
        for (int h = 0; h < 24; h++) {
            robotPoses.add(new Pose2d(0.2, -0.1, Rotation2d.fromDegrees(h * 15)));
        }
    }

    ExtrinsicDataset build() {
        List<ExtrinsicDataset.CameraInfo> cams = new ArrayList<>();
        for (var name : trueRobotToCamera.keySet()) {
            cams.add(new ExtrinsicDataset.CameraInfo(name, name, WIDTH, HEIGHT, K.clone(), DIST.clone()));
        }

        Matrix<N3, N3> cameraMatrix = new Matrix<>(new SimpleMatrix(3, 3, true, K));
        Matrix<N8, N1> distCoeffs = new Matrix<>(new SimpleMatrix(8, 1, true, DIST));
        var model = new TargetModel(TAG_SIZE, TAG_SIZE);

        List<ExtrinsicDataset.Stop> stops = new ArrayList<>();
        Pose2d odom = new Pose2d(1.3, -2.1, Rotation2d.fromDegrees(40));
        for (int k = 0; k < robotPoses.size(); k++) {
            Pose2d truth = robotPoses.get(k);
            if (k > 0) {
                Transform2d delta = truth.minus(robotPoses.get(k - 1));
                double d = delta.getTranslation().getNorm();
                double sigmaT = odomTransNoise + odomTransNoisePerMeter * d;
                var noisy =
                        new Transform2d(
                                delta.getX() * odomScale + random.nextGaussian() * sigmaT,
                                delta.getY() * odomScale + random.nextGaussian() * sigmaT,
                                delta.getRotation().plus(new Rotation2d(random.nextGaussian() * odomRotNoise)));
                odom = odom.plus(noisy);
            }

            Pose3d robot3d = new Pose3d(truth);
            List<ExtrinsicDataset.TagObservation> obs = new ArrayList<>();
            for (var camEntry : trueRobotToCamera.entrySet()) {
                Pose3d camPose = robot3d.transformBy(camEntry.getValue());
                var camRt = RotTrlTransform3d.makeRelativeTo(camPose);
                for (var tagEntry : trueTags.entrySet()) {
                    Pose3d tagPose = tagEntry.getValue();
                    if (!isVisible(camPose, tagPose)) continue;
                    var pts =
                            OpenCVHelp.projectPoints(
                                    cameraMatrix, distCoeffs, camRt, model.getFieldVertices(tagPose));
                    double[] corners = new double[8];
                    boolean inFrame = true;
                    for (int i = 0; i < 4; i++) {
                        corners[2 * i] = pts[i].x + random.nextGaussian() * pixelNoise;
                        corners[2 * i + 1] = pts[i].y + random.nextGaussian() * pixelNoise;
                        if (pts[i].x < 5 || pts[i].x > WIDTH - 5 || pts[i].y < 5 || pts[i].y > HEIGHT - 5)
                            inFrame = false;
                    }
                    if (inFrame)
                        obs.add(
                                new ExtrinsicDataset.TagObservation(
                                        camEntry.getKey(), tagEntry.getKey(), corners, 5));
                }
            }
            stops.add(
                    new ExtrinsicDataset.Stop(
                            k * 2.0, odom.getX(), odom.getY(), odom.getRotation().getRadians(), obs));
        }
        return new ExtrinsicDataset(ExtrinsicDataset.CURRENT_VERSION, TAG_SIZE, cams, stops);
    }

    private static boolean isVisible(Pose3d camPose, Pose3d tagPose) {
        Translation3d toTagInCam =
                tagPose
                        .getTranslation()
                        .minus(camPose.getTranslation())
                        .rotateBy(camPose.getRotation().inverse());
        if (toTagInCam.getX() < 0.3 || toTagInCam.getNorm() > 6) return false;
        // Tag normal (+X) must point back toward the camera within 75 degrees
        Translation3d normal = new Translation3d(1, 0, 0).rotateBy(tagPose.getRotation());
        Translation3d toCam = camPose.getTranslation().minus(tagPose.getTranslation());
        double cos =
                (normal.getX() * toCam.getX() + normal.getY() * toCam.getY() + normal.getZ() * toCam.getZ())
                        / toCam.getNorm();
        return cos > Math.cos(Math.toRadians(75));
    }
}
