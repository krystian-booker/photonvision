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
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeMap;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;
import org.ejml.simple.SimpleMatrix;
import org.opencv.core.Point;
import org.photonvision.common.logging.LogGroup;
import org.photonvision.common.logging.Logger;
import org.photonvision.estimation.OpenCVHelp;
import org.photonvision.estimation.TargetModel;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import org.wpilib.math.numbers.N8;

/**
 * Solves for the robot-to-camera transform of every camera on a robot from an {@link
 * ExtrinsicDataset}.
 *
 * <p>The robot frame is defined the same way robot code defines it: X/Y/yaw by the drivetrain
 * (odometry), and Z/roll/pitch by the floor. The problem is a small factor graph solved with
 * Levenberg-Marquardt:
 *
 * <ul>
 *   <li>Unknowns: robot pose at each stop (planar x, y, theta), robot-to-camera transform of each
 *       camera (6 DoF), tag poses (6 DoF, 3 DoF for tags lying on the floor, or fixed when using a
 *       known field layout), and optionally an odometry scale factor.
 *   <li>Tag corner reprojection factors (robust Huber loss) using each camera's existing intrinsic
 *       calibration.
 *   <li>Odometry between-factors relating consecutive stops.
 * </ul>
 *
 * <p>Observability: camera x/y come from spinning the robot, camera yaw from driving it in straight
 * lines, and camera z/roll/pitch from the floor datum (floor tags or a known field layout). The
 * marginal covariance of every camera is reported so the UI can tell users what's missing.
 */
public final class ExtrinsicSolver {
    private static final Logger logger = new Logger(ExtrinsicSolver.class, LogGroup.General);

    // Robot rotation between two stops above which we use the pair to find the camera's up axis
    private static final double MIN_AXIS_ROTATION_RAD = Math.toRadians(10);
    // Single tag PnP solutions more ambiguous than this are not trusted for initialization
    private static final double MAX_INIT_AMBIGUITY = 0.4;
    // Uncertainty thresholds beyond which we warn the user
    private static final double WARN_TRANSLATION_STDDEV_M = 0.01;
    private static final double WARN_ROTATION_STDDEV_DEG = 0.5;

    private ExtrinsicSolver() {}

    /**
     * Solve for every camera's robot-to-camera transform.
     *
     * @param dataset Recorded stops and observations
     * @param settings Solver settings
     * @param knownLayout Tag poses to hold fixed. Required when {@link
     *     ExtrinsicSolverSettings#useFieldLayout()} is set, ignored otherwise.
     * @return The result. Check {@link ExtrinsicSolveResult#success()}.
     */
    public static ExtrinsicSolveResult solve(
            ExtrinsicDataset dataset,
            ExtrinsicSolverSettings settings,
            Map<Integer, Pose3d> knownLayout) {
        try {
            return new Problem(dataset, settings, knownLayout).run();
        } catch (SolverException e) {
            logger.warn("Extrinsic calibration failed: " + e.getMessage());
            return ExtrinsicSolveResult.failure(e.getMessage());
        }
    }

    static final class SolverException extends Exception {
        SolverException(String message) {
            super(message);
        }
    }

    // ------------------------------------------------------------------------------------------
    // Projection
    // ------------------------------------------------------------------------------------------

    /**
     * Projects a point in the WPILib camera frame (X forward, Y left, Z up) to distorted pixel
     * coordinates using the OpenCV camera model.
     *
     * @return false if the point is behind the camera
     */
    static boolean project(double[] K, double[] dist, double[] pNwu, double[] outUv) {
        // NWU -> EDN (OpenCV)
        double x = -pNwu[1];
        double y = -pNwu[2];
        double z = pNwu[0];
        if (z < 1e-6) return false;
        double xp = x / z;
        double yp = y / z;
        double r2 = xp * xp + yp * yp;
        double k1 = dist.length > 0 ? dist[0] : 0;
        double k2 = dist.length > 1 ? dist[1] : 0;
        double p1 = dist.length > 2 ? dist[2] : 0;
        double p2 = dist.length > 3 ? dist[3] : 0;
        double k3 = dist.length > 4 ? dist[4] : 0;
        double k4 = dist.length > 5 ? dist[5] : 0;
        double k5 = dist.length > 6 ? dist[6] : 0;
        double k6 = dist.length > 7 ? dist[7] : 0;
        double radial = (1 + r2 * (k1 + r2 * (k2 + r2 * k3))) / (1 + r2 * (k4 + r2 * (k5 + r2 * k6)));
        double xpp = xp * radial + 2 * p1 * xp * yp + p2 * (r2 + 2 * xp * xp);
        double ypp = yp * radial + p1 * (r2 + 2 * yp * yp) + 2 * p2 * xp * yp;
        outUv[0] = K[0] * xpp + K[1] * ypp + K[2];
        outUv[1] = K[4] * ypp + K[5];
        return true;
    }

    static double wrapAngle(double a) {
        return Math.atan2(Math.sin(a), Math.cos(a));
    }

    // ------------------------------------------------------------------------------------------
    // Parameter blocks
    // ------------------------------------------------------------------------------------------

    private abstract static class Block {
        /** Index of this block's first coordinate in the global parameter vector, -1 if constant. */
        int offset = -1;

        abstract int dim();

        /** Perturb local coordinate i by h. Perturbing by h then -h must restore the state. */
        abstract void step(int i, double h);

        abstract double[] save();

        abstract void restore(double[] s);

        void apply(double[] delta) {
            for (int i = 0; i < dim(); i++) step(i, delta[offset + i]);
        }

        boolean isFree() {
            return offset >= 0;
        }
    }

    /**
     * A free 6 DoF pose: t += v, R = Exp(w) * R. If {@code lockZ} is set the height is held constant
     * and the block has 5 DoF (used for tags at a measured height).
     */
    private static final class PoseBlock extends Block {
        Rigid3 T;
        final boolean lockZ;

        PoseBlock(Rigid3 T) {
            this(T, false);
        }

        PoseBlock(Rigid3 T, boolean lockZ) {
            this.T = T;
            this.lockZ = lockZ;
        }

        @Override
        int dim() {
            return lockZ ? 5 : 6;
        }

        @Override
        void step(int i, double h) {
            int nT = lockZ ? 2 : 3;
            if (i < nT) {
                T.t[i] += h;
            } else {
                int axis = i - nT;
                T.rotateLeft(axis == 0 ? h : 0, axis == 1 ? h : 0, axis == 2 ? h : 0);
            }
        }

        @Override
        void apply(double[] d) {
            int nT = lockZ ? 2 : 3;
            for (int i = 0; i < nT; i++) T.t[i] += d[offset + i];
            T.rotateLeft(d[offset + nT], d[offset + nT + 1], d[offset + nT + 2]);
        }

        @Override
        double[] save() {
            double[] s = new double[12];
            System.arraycopy(T.R, 0, s, 0, 9);
            System.arraycopy(T.t, 0, s, 9, 3);
            return s;
        }

        @Override
        void restore(double[] s) {
            System.arraycopy(s, 0, T.R, 0, 9);
            System.arraycopy(s, 9, T.t, 0, 3);
        }
    }

    /** A planar robot pose (x, y, theta) on the floor. */
    private static final class PlanarBlock extends Block {
        final double[] p = new double[3];

        PlanarBlock(double x, double y, double theta) {
            p[0] = x;
            p[1] = y;
            p[2] = theta;
        }

        Rigid3 pose() {
            return Rigid3.planar(p[0], p[1], p[2]);
        }

        @Override
        int dim() {
            return 3;
        }

        @Override
        void step(int i, double h) {
            p[i] += h;
        }

        @Override
        double[] save() {
            return p.clone();
        }

        @Override
        void restore(double[] s) {
            System.arraycopy(s, 0, p, 0, 3);
        }
    }

    /** A tag lying flat on the floor, face up: x, y, and rotation about the floor normal. */
    private static final class FloorTagBlock extends Block {
        final double[] p = new double[3];
        final double height;

        FloorTagBlock(double x, double y, double psi, double height) {
            p[0] = x;
            p[1] = y;
            p[2] = psi;
            this.height = height;
        }

        Rigid3 pose() {
            // Tag +X (out of the face) points up
            return new Rigid3(
                    Rigid3.mul(Rigid3.rotZ(p[2]), Rigid3.rotY(-Math.PI / 2)),
                    new double[] {p[0], p[1], height});
        }

        @Override
        int dim() {
            return 3;
        }

        @Override
        void step(int i, double h) {
            p[i] += h;
        }

        @Override
        double[] save() {
            return p.clone();
        }

        @Override
        void restore(double[] s) {
            System.arraycopy(s, 0, p, 0, 3);
        }
    }

    private static final class ScalarBlock extends Block {
        double v;

        ScalarBlock(double v) {
            this.v = v;
        }

        @Override
        int dim() {
            return 1;
        }

        @Override
        void step(int i, double h) {
            v += h;
        }

        @Override
        double[] save() {
            return new double[] {v};
        }

        @Override
        void restore(double[] s) {
            v = s[0];
        }
    }

    // ------------------------------------------------------------------------------------------
    // Problem entities
    // ------------------------------------------------------------------------------------------

    private static final class Cam {
        final ExtrinsicDataset.CameraInfo info;
        final double[] K;
        final double[] dist;
        final Matrix<N3, N3> cameraMatrix;
        final Matrix<N8, N1> distCoeffs;
        PoseBlock X;
        final List<Obs> obs = new ArrayList<>();

        Cam(ExtrinsicDataset.CameraInfo info) {
            this.info = info;
            this.K = info.cameraMatrix();
            this.dist = info.distCoeffs() == null ? new double[0] : info.distCoeffs();
            this.cameraMatrix = new Matrix<>(new SimpleMatrix(3, 3, true, K));
            double[] d8 = Arrays.copyOf(dist, 8);
            this.distCoeffs = new Matrix<>(new SimpleMatrix(8, 1, true, d8));
        }
    }

    private static final class StopState {
        final int index;
        final ExtrinsicDataset.Stop stop;
        PlanarBlock pose;

        StopState(int index, ExtrinsicDataset.Stop stop) {
            this.index = index;
            this.stop = stop;
        }

        Pose2d odom() {
            return new Pose2d(stop.odomX(), stop.odomY(), new Rotation2d(stop.odomThetaRad()));
        }
    }

    private static final class Tag {
        final int id;
        final boolean floor;
        final boolean fixed;
        // Measured height of the tag's center, or null
        final Double knownHeight;
        Block block;
        int observations = 0;

        Tag(int id, boolean floor, boolean fixed, Double knownHeight) {
            this.id = id;
            this.floor = floor;
            this.fixed = fixed;
            this.knownHeight = knownHeight;
        }

        Rigid3 pose() {
            if (block instanceof FloorTagBlock f) return f.pose();
            return ((PoseBlock) block).T;
        }
    }

    private static final class Obs {
        final Cam cam;
        final StopState stop;
        final Tag tag;
        final double[] uv;
        // Camera-to-tag from single tag PnP, null if unavailable or ambiguous
        Rigid3 pnp;

        Obs(Cam cam, StopState stop, Tag tag, double[] uv) {
            this.cam = cam;
            this.stop = stop;
            this.tag = tag;
            this.uv = uv;
        }
    }

    // ------------------------------------------------------------------------------------------
    // Factors
    // ------------------------------------------------------------------------------------------

    private abstract static class Factor {
        final Block[] blocks;

        Factor(Block... blocks) {
            this.blocks = blocks;
        }

        abstract int residualDim();

        abstract void evaluate(double[] r);

        /** Robust cost of a residual vector and the IRLS weight of each residual. */
        double cost(double[] r, double[] w) {
            double c = 0;
            for (int i = 0; i < r.length; i++) {
                c += r[i] * r[i];
                w[i] = 1;
            }
            return c;
        }
    }

    private static final class PriorFactor extends Factor {
        final ScalarBlock block;
        final double mean;
        final double sigma;

        PriorFactor(ScalarBlock block, double mean, double sigma) {
            super(block);
            this.block = block;
            this.mean = mean;
            this.sigma = sigma;
        }

        @Override
        int residualDim() {
            return 1;
        }

        @Override
        void evaluate(double[] r) {
            r[0] = (block.v - mean) / sigma;
        }
    }

    // ------------------------------------------------------------------------------------------
    // The problem
    // ------------------------------------------------------------------------------------------

    private static final class Problem {
        final ExtrinsicDataset dataset;
        final ExtrinsicSolverSettings settings;
        final Map<Integer, Pose3d> layout;
        final double[][] tagCorners = new double[4][];
        final List<Translation3d> tagModel;

        final Map<String, Cam> cams = new LinkedHashMap<>();
        final List<StopState> stops = new ArrayList<>();
        final Map<Integer, Tag> tags = new TreeMap<>();
        final List<Obs> observations = new ArrayList<>();
        final ScalarBlock scale = new ScalarBlock(1.0);
        final List<Block> blocks = new ArrayList<>();
        final List<Factor> factors = new ArrayList<>();
        final List<String> warnings = new ArrayList<>();
        int numParams = 0;

        Problem(
                ExtrinsicDataset dataset, ExtrinsicSolverSettings settings, Map<Integer, Pose3d> layout) {
            this.dataset = dataset;
            this.settings = settings;
            this.layout = layout;
            this.tagModel = new TargetModel(dataset.tagSizeMeters(), dataset.tagSizeMeters()).vertices;
            for (int i = 0; i < 4; i++) {
                var v = tagModel.get(i);
                tagCorners[i] = new double[] {v.getX(), v.getY(), v.getZ()};
            }
        }

        ExtrinsicSolveResult run() throws SolverException {
            buildEntities();
            runPnp();
            initRobotPosesFromOdometry();
            for (var cam : cams.values()) initCameraRotationAndXY(cam);
            if (settings.useFieldLayout()) {
                initWithKnownLayout();
            } else {
                initSelfSurvey();
            }
            buildParameters();
            buildFactors();
            int iterations = optimize();
            return buildResult(iterations);
        }

        private final class ReprojectionFactor extends Factor {
            final Obs obs;
            final double invSigma;
            final double huber;
            private final double[] pw = new double[3];
            private final double[] pc = new double[3];
            private final double[] uv = new double[2];

            ReprojectionFactor(Obs obs, double sigma, double huberPx) {
                super(obs.stop.pose, obs.cam.X, obs.tag.block);
                this.obs = obs;
                this.invSigma = 1.0 / sigma;
                this.huber = huberPx / sigma;
            }

            @Override
            int residualDim() {
                return 8;
            }

            @Override
            void evaluate(double[] r) {
                Rigid3 worldToCam = obs.stop.pose.pose().compose(obs.cam.X.T).inverse();
                Rigid3 tagPose = obs.tag.pose();
                for (int i = 0; i < 4; i++) {
                    tagPose.transformPoint(tagCorners[i], pw);
                    worldToCam.transformPoint(pw, pc);
                    if (project(obs.cam.K, obs.cam.dist, pc, uv)) {
                        r[2 * i] = (uv[0] - obs.uv[2 * i]) * invSigma;
                        r[2 * i + 1] = (uv[1] - obs.uv[2 * i + 1]) * invSigma;
                    } else {
                        // Behind the camera: a large but finite residual keeps the optimizer moving
                        r[2 * i] = 1e3;
                        r[2 * i + 1] = 1e3;
                    }
                }
            }

            @Override
            double cost(double[] r, double[] w) {
                double c = 0;
                for (int i = 0; i < 4; i++) {
                    double e = Math.hypot(r[2 * i], r[2 * i + 1]);
                    double wi;
                    if (e <= huber) {
                        c += e * e;
                        wi = 1;
                    } else {
                        c += 2 * huber * e - huber * huber;
                        wi = huber / e;
                    }
                    w[2 * i] = wi;
                    w[2 * i + 1] = wi;
                }
                return c;
            }
        }

        private final class OdometryFactor extends Factor {
            final StopState a;
            final StopState b;
            final ScalarBlock scaleBlock;
            final double dx;
            final double dy;
            final double dtheta;
            final double sigmaT;
            final double sigmaR;

            OdometryFactor(StopState a, StopState b, ScalarBlock scale) {
                super(a.pose, b.pose, scale);
                this.a = a;
                this.b = b;
                this.scaleBlock = scale;
                var delta = b.odom().relativeTo(a.odom());
                dx = delta.getX();
                dy = delta.getY();
                dtheta = delta.getRotation().getRadians();
                sigmaT =
                        settings.odomTranslationStdDev()
                                + settings.odomTranslationStdDevPerMeter() * Math.hypot(dx, dy);
                sigmaR =
                        settings.odomRotationStdDev()
                                + settings.odomRotationStdDevPerRadian() * Math.abs(dtheta);
            }

            @Override
            int residualDim() {
                return 3;
            }

            @Override
            void evaluate(double[] r) {
                double[] pa = a.pose.p;
                double[] pb = b.pose.p;
                double c = Math.cos(pa[2]);
                double s = Math.sin(pa[2]);
                double ex = pb[0] - pa[0];
                double ey = pb[1] - pa[1];
                double px = c * ex + s * ey;
                double py = -s * ex + c * ey;
                double k = scaleBlock.v;
                r[0] = (px - k * dx) / sigmaT;
                r[1] = (py - k * dy) / sigmaT;
                r[2] = wrapAngle(pb[2] - pa[2] - dtheta) / sigmaR;
            }
        }

        // -------------------------------- setup --------------------------------

        void buildEntities() throws SolverException {
            if (dataset.tagSizeMeters() <= 0) throw new SolverException("Tag size must be positive");
            if (dataset.cameras() == null || dataset.cameras().isEmpty())
                throw new SolverException("No cameras in dataset");
            if (dataset.stops() == null || dataset.stops().size() < 3)
                throw new SolverException(
                        "Need at least 3 robot stops, got "
                                + (dataset.stops() == null ? 0 : dataset.stops().size()));
            if (settings.useFieldLayout() && (layout == null || layout.isEmpty()))
                throw new SolverException("Field layout mode selected but no layout is available");

            for (var info : dataset.cameras()) {
                if (info.cameraMatrix() == null || info.cameraMatrix().length != 9)
                    throw new SolverException("Camera " + info.nickname() + " has no intrinsic calibration");
                cams.put(info.uniqueName(), new Cam(info));
            }

            Set<Integer> floorIds =
                    settings.floorTagIds() == null ? Set.of() : new HashSet<>(settings.floorTagIds());
            Map<Integer, Double> knownHeights = new HashMap<>();
            if (settings.knownTagHeights() != null) {
                for (var th : settings.knownTagHeights()) knownHeights.put(th.id(), th.heightMeters());
            }
            for (int i = 0; i < dataset.stops().size(); i++) {
                var stopState = new StopState(i, dataset.stops().get(i));
                stops.add(stopState);
                if (stopState.stop.observations() == null) continue;
                for (var o : stopState.stop.observations()) {
                    var cam = cams.get(o.camera());
                    if (cam == null || o.corners() == null || o.corners().length != 8) continue;
                    if (Arrays.stream(o.corners()).anyMatch(v -> !Double.isFinite(v))) continue;
                    Tag tag;
                    if (settings.useFieldLayout()) {
                        if (!layout.containsKey(o.tagId())) continue;
                        tag = tags.computeIfAbsent(o.tagId(), id -> new Tag(id, false, true, null));
                    } else {
                        tag =
                                tags.computeIfAbsent(
                                        o.tagId(),
                                        id ->
                                                new Tag(
                                                        id,
                                                        floorIds.contains(id),
                                                        false,
                                                        floorIds.contains(id) ? null : knownHeights.get(id)));
                    }
                    var obs = new Obs(cam, stopState, tag, o.corners());
                    observations.add(obs);
                    cam.obs.add(obs);
                    tag.observations++;
                }
            }
            if (observations.isEmpty()) throw new SolverException("No usable tag observations");

            if (!settings.useFieldLayout()
                    && tags.values().stream().noneMatch(t -> t.floor || t.knownHeight != null)) {
                throw new SolverException(
                        "Self-survey mode needs at least one observed tag lying flat on the floor, or a "
                                + "tag with a measured height, to define camera heights");
            }
            for (var cam : cams.values()) {
                if (cam.obs.isEmpty()) {
                    warnings.add(cam.info.nickname() + " saw no tags and was skipped");
                }
            }
            cams.values().removeIf(c -> c.obs.isEmpty());
        }

        void runPnp() {
            for (var obs : observations) {
                Point[] pts = new Point[4];
                for (int i = 0; i < 4; i++) pts[i] = new Point(obs.uv[2 * i], obs.uv[2 * i + 1]);
                var res =
                        OpenCVHelp.solvePNP_SQUARE(obs.cam.cameraMatrix, obs.cam.distCoeffs, tagModel, pts);
                if (res.isPresent() && res.get().ambiguity < MAX_INIT_AMBIGUITY) {
                    obs.pnp = Rigid3.fromTransform(res.get().best);
                }
            }
        }

        void initRobotPosesFromOdometry() {
            Pose2d origin = stops.get(0).odom();
            for (var s : stops) {
                var rel = s.odom().relativeTo(origin);
                s.pose = new PlanarBlock(rel.getX(), rel.getY(), rel.getRotation().getRadians());
            }
        }

        /**
         * Planar hand-eye initialization. For a tag seen from stops a and b: A X = X B, with A the
         * planar robot motion from odometry and B the camera motion from PnP. The camera's up axis
         * comes from B's rotation axis, then x, y and yaw from a linear least squares problem.
         */
        void initCameraRotationAndXY(Cam cam) throws SolverException {
            Map<Integer, List<Obs>> byTag = new TreeMap<>();
            for (var o : cam.obs) {
                if (o.pnp != null) byTag.computeIfAbsent(o.tag.id, k -> new ArrayList<>()).add(o);
            }
            List<Obs[]> pairs = new ArrayList<>();
            for (var list : byTag.values()) {
                for (int i = 0; i < list.size(); i++) {
                    for (int j = i + 1; j < Math.min(list.size(), i + 16); j++) {
                        if (list.get(i).stop != list.get(j).stop)
                            pairs.add(new Obs[] {list.get(i), list.get(j)});
                    }
                }
            }

            // Camera up axis: the rotation axis of B, scaled so that it maps to +Z
            double[] axis = new double[3];
            for (var p : pairs) {
                double thetaA = relativeHeading(p[0].stop, p[1].stop);
                if (Math.abs(thetaA) < MIN_AXIS_ROTATION_RAD) continue;
                Rigid3 B = p[0].pnp.compose(p[1].pnp.inverse());
                double[] w = Rigid3.logSO3(B.R);
                double sgn = Math.signum(thetaA);
                axis[0] += w[0] * sgn;
                axis[1] += w[1] * sgn;
                axis[2] += w[2] * sgn;
            }
            double norm = Math.sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
            if (norm < 1e-6) {
                throw new SolverException(
                        cam.info.nickname()
                                + ": the robot never turned more than "
                                + Math.round(Math.toDegrees(MIN_AXIS_ROTATION_RAD))
                                + " degrees while this camera saw the same tag. Rotate the robot in place.");
            }
            double[] b = {axis[0] / norm, axis[1] / norm, axis[2] / norm};
            double[] R0 = rotationTakingToZ(b);

            // Solve (R_A - I) t_xy - Rz2(psi) (R0 t_B)_xy = -t_A for (tx, ty, cos psi, sin psi)
            DMatrixRMaj AtA = new DMatrixRMaj(4, 4);
            DMatrixRMaj Atb = new DMatrixRMaj(4, 1);
            double[] u = new double[3];
            for (var p : pairs) {
                var A = relativePlanar(p[0].stop, p[1].stop);
                Rigid3 B = p[0].pnp.compose(p[1].pnp.inverse());
                Rigid3.apply(R0, B.t, u);
                double c = Math.cos(A[2]);
                double s = Math.sin(A[2]);
                double[][] rows = {
                    {c - 1, -s, -u[0], u[1]},
                    {s, c - 1, -u[1], -u[0]}
                };
                double[] rhs = {-A[0], -A[1]};
                for (int r = 0; r < 2; r++) {
                    for (int i = 0; i < 4; i++) {
                        Atb.add(i, 0, rows[r][i] * rhs[r]);
                        for (int j = 0; j < 4; j++) AtA.add(i, j, rows[r][i] * rows[r][j]);
                    }
                }
            }
            for (int i = 0; i < 4; i++) AtA.add(i, i, 1e-9);
            DMatrixRMaj sol = new DMatrixRMaj(4, 1);
            if (!CommonOps_DDRM.solve(AtA, Atb, sol)) {
                throw new SolverException(cam.info.nickname() + ": could not initialize camera pose");
            }
            double cs = Math.hypot(sol.get(2), sol.get(3));
            if (cs < 0.3) {
                throw new SolverException(
                        cam.info.nickname()
                                + ": camera yaw is unobservable. Drive the robot in straight lines "
                                + "(forward/back and sideways) between stops, not just spinning in place.");
            }
            double psi = Math.atan2(sol.get(3), sol.get(2));
            cam.X =
                    new PoseBlock(
                            new Rigid3(
                                    Rigid3.mul(Rigid3.rotZ(psi), R0), new double[] {sol.get(0), sol.get(1), 0}));
        }

        /** Odometry-predicted motion from stop a to b, expressed in a's frame: (x, y, theta). */
        double[] relativePlanar(StopState a, StopState b) {
            double[] pa = a.pose.p;
            double[] pb = b.pose.p;
            double c = Math.cos(pa[2]);
            double s = Math.sin(pa[2]);
            double ex = pb[0] - pa[0];
            double ey = pb[1] - pa[1];
            return new double[] {c * ex + s * ey, -s * ex + c * ey, wrapAngle(pb[2] - pa[2])};
        }

        double relativeHeading(StopState a, StopState b) {
            return wrapAngle(b.pose.p[2] - a.pose.p[2]);
        }

        /** The minimal rotation R such that R * b = +Z. */
        static double[] rotationTakingToZ(double[] b) {
            // axis = b x z, angle = acos(b . z)
            double ax = b[1];
            double ay = -b[0];
            double s = Math.hypot(ax, ay);
            double c = b[2];
            if (s < 1e-9) {
                return c > 0 ? Rigid3.identity().R : Rigid3.expSO3(Math.PI, 0, 0);
            }
            double angle = Math.atan2(s, c);
            return Rigid3.expSO3(ax / s * angle, ay / s * angle, 0);
        }

        /** World pose of a tag implied by one observation, given current robot and camera poses. */
        Rigid3 tagFromObs(Obs o) {
            return o.stop.pose.pose().compose(o.cam.X.T).compose(o.pnp);
        }

        void initSelfSurvey() throws SolverException {
            // Camera heights are unobservable from planar motion. Anchor them to the floor tags, then
            // propagate through tags shared between cameras.
            Map<Cam, Map<Integer, Double>> tagZByCam = new HashMap<>();
            for (var cam : cams.values()) {
                Map<Integer, List<Double>> zs = new HashMap<>();
                for (var o : cam.obs) {
                    if (o.pnp != null)
                        zs.computeIfAbsent(o.tag.id, k -> new ArrayList<>()).add(tagFromObs(o).t[2]);
                }
                Map<Integer, Double> med = new HashMap<>();
                zs.forEach((id, list) -> med.put(id, median(list)));
                tagZByCam.put(cam, med);
            }
            Map<Integer, Double> tagZ = new HashMap<>();
            for (var t : tags.values()) {
                if (t.floor) tagZ.put(t.id, settings.floorTagHeightMeters());
                else if (t.knownHeight != null) tagZ.put(t.id, t.knownHeight);
            }
            Set<Cam> anchored = new HashSet<>();
            boolean progress = true;
            while (progress) {
                progress = false;
                for (var cam : cams.values()) {
                    if (anchored.contains(cam)) continue;
                    List<Double> offsets = new ArrayList<>();
                    tagZByCam
                            .get(cam)
                            .forEach(
                                    (id, z) -> {
                                        if (tagZ.containsKey(id)) offsets.add(tagZ.get(id) - z);
                                    });
                    if (offsets.isEmpty()) continue;
                    cam.X.T.t[2] = median(offsets);
                    anchored.add(cam);
                    progress = true;
                    tagZByCam.get(cam).forEach((id, z) -> tagZ.putIfAbsent(id, z + cam.X.T.t[2]));
                }
            }
            for (var cam : cams.values()) {
                if (!anchored.contains(cam)) {
                    throw new SolverException(
                            cam.info.nickname()
                                    + " never saw a floor tag, a tag with a measured height, or a tag shared "
                                    + "with a camera that did, "
                                    + "so its height can't be determined");
                }
            }

            // Tag poses: mean translation, rotation from the closest observation
            for (var tag : tags.values()) {
                double[] mean = new double[3];
                int n = 0;
                Rigid3 best = null;
                double bestDist = Double.MAX_VALUE;
                for (var o : observations) {
                    if (o.tag != tag || o.pnp == null) continue;
                    Rigid3 w = tagFromObs(o);
                    for (int i = 0; i < 3; i++) mean[i] += w.t[i];
                    n++;
                    double d =
                            Math.sqrt(
                                    o.pnp.t[0] * o.pnp.t[0] + o.pnp.t[1] * o.pnp.t[1] + o.pnp.t[2] * o.pnp.t[2]);
                    if (d < bestDist) {
                        bestDist = d;
                        best = w;
                    }
                }
                if (n == 0) {
                    // Only ambiguous observations. Drop the tag.
                    tag.block = null;
                    continue;
                }
                for (int i = 0; i < 3; i++) mean[i] /= n;
                if (tag.floor) {
                    double psi = Math.atan2(-best.R[1], best.R[4]);
                    tag.block = new FloorTagBlock(mean[0], mean[1], psi, settings.floorTagHeightMeters());
                } else if (tag.knownHeight != null) {
                    mean[2] = tag.knownHeight;
                    tag.block = new PoseBlock(new Rigid3(best.R, mean), true);
                } else {
                    tag.block = new PoseBlock(new Rigid3(best.R, mean));
                }
            }
            observations.removeIf(o -> o.tag.block == null);
            tags.values().removeIf(t -> t.block == null);
        }

        void initWithKnownLayout() throws SolverException {
            for (var tag : tags.values()) {
                tag.block = new PoseBlock(Rigid3.fromPose(layout.get(tag.id)));
            }
            // Camera height: the camera's world z, since the robot sits on the field floor
            for (var cam : cams.values()) {
                List<Double> zs = new ArrayList<>();
                for (var o : cam.obs) {
                    if (o.pnp != null) zs.add(o.tag.pose().compose(o.pnp.inverse()).t[2]);
                }
                if (zs.isEmpty())
                    throw new SolverException(cam.info.nickname() + " has no unambiguous tag observations");
                cam.X.T.t[2] = median(zs);
            }
            // Robot poses: from tag observations where available, otherwise chained from odometry
            double[][] fromTags = new double[stops.size()][];
            for (var s : stops) {
                double sx = 0, sy = 0, sc = 0, ss = 0;
                int n = 0;
                for (var o : observations) {
                    if (o.stop != s || o.pnp == null) continue;
                    Rigid3 robot = o.tag.pose().compose(o.pnp.inverse()).compose(o.cam.X.T.inverse());
                    sx += robot.t[0];
                    sy += robot.t[1];
                    double yaw = robot.yaw();
                    sc += Math.cos(yaw);
                    ss += Math.sin(yaw);
                    n++;
                }
                if (n > 0) fromTags[s.index] = new double[] {sx / n, sy / n, Math.atan2(ss, sc)};
            }
            int first = -1;
            for (int i = 0; i < stops.size(); i++) {
                if (fromTags[i] != null) {
                    first = i;
                    break;
                }
            }
            if (first < 0) throw new SolverException("No stop could be located in the field layout");
            double[][] odomRel = new double[stops.size()][];
            for (var s : stops) odomRel[s.index] = s.pose.p.clone();
            double[] anchor = fromTags[first];
            int anchorIdx = first;
            for (int i = 0; i < stops.size(); i++) {
                if (fromTags[i] != null) {
                    anchor = fromTags[i];
                    anchorIdx = i;
                    System.arraycopy(fromTags[i], 0, stops.get(i).pose.p, 0, 3);
                } else {
                    // anchor * (odom[anchorIdx]^-1 * odom[i])
                    Rigid3 rel =
                            Rigid3.planar(odomRel[anchorIdx][0], odomRel[anchorIdx][1], odomRel[anchorIdx][2])
                                    .inverse()
                                    .compose(Rigid3.planar(odomRel[i][0], odomRel[i][1], odomRel[i][2]));
                    Rigid3 world = Rigid3.planar(anchor[0], anchor[1], anchor[2]).compose(rel);
                    stops.get(i).pose.p[0] = world.t[0];
                    stops.get(i).pose.p[1] = world.t[1];
                    stops.get(i).pose.p[2] = world.yaw();
                }
            }
        }

        void buildParameters() {
            boolean fixFirstStop = !settings.useFieldLayout();
            for (var s : stops) {
                if (fixFirstStop && s.index == 0) continue;
                addBlock(s.pose);
            }
            for (var cam : cams.values()) addBlock(cam.X);
            for (var tag : tags.values()) if (!tag.fixed) addBlock(tag.block);
            if (settings.estimateOdometryScale()) addBlock(scale);
        }

        void addBlock(Block b) {
            b.offset = numParams;
            numParams += b.dim();
            blocks.add(b);
        }

        void buildFactors() {
            for (var o : observations) {
                factors.add(new ReprojectionFactor(o, settings.pixelStdDev(), settings.huberThresholdPx()));
            }
            for (int i = 0; i + 1 < stops.size(); i++) {
                factors.add(new OdometryFactor(stops.get(i), stops.get(i + 1), scale));
            }
            if (settings.estimateOdometryScale()) factors.add(new PriorFactor(scale, 1.0, 0.05));
        }

        // -------------------------------- optimization --------------------------------

        /** Linearizes all factors into H = J^T W J and g = J^T W r. Returns the robust cost. */
        double linearize(DMatrixRMaj H, DMatrixRMaj g) {
            H.zero();
            g.zero();
            double total = 0;
            final double h = 1e-6;
            for (var f : factors) {
                int m = f.residualDim();
                double[] r = new double[m];
                double[] w = new double[m];
                f.evaluate(r);
                total += f.cost(r, w);

                int n = 0;
                for (var b : f.blocks) if (b.isFree()) n += b.dim();
                if (n == 0) continue;
                int[] idx = new int[n];
                double[][] J = new double[n][];
                double[] rp = new double[m];
                double[] rm = new double[m];
                int col = 0;
                for (var b : f.blocks) {
                    if (!b.isFree()) continue;
                    for (int i = 0; i < b.dim(); i++) {
                        b.step(i, h);
                        f.evaluate(rp);
                        b.step(i, -2 * h);
                        f.evaluate(rm);
                        b.step(i, h);
                        double[] c = new double[m];
                        for (int k = 0; k < m; k++) c[k] = (rp[k] - rm[k]) / (2 * h);
                        J[col] = c;
                        idx[col] = b.offset + i;
                        col++;
                    }
                }
                for (int a = 0; a < n; a++) {
                    double ga = 0;
                    for (int k = 0; k < m; k++) ga += J[a][k] * w[k] * r[k];
                    g.add(idx[a], 0, ga);
                    for (int bb = a; bb < n; bb++) {
                        double hab = 0;
                        for (int k = 0; k < m; k++) hab += J[a][k] * w[k] * J[bb][k];
                        H.add(idx[a], idx[bb], hab);
                        if (bb != a) H.add(idx[bb], idx[a], hab);
                    }
                }
            }
            return total;
        }

        double totalCost() {
            double total = 0;
            for (var f : factors) {
                double[] r = new double[f.residualDim()];
                double[] w = new double[f.residualDim()];
                f.evaluate(r);
                total += f.cost(r, w);
            }
            return total;
        }

        List<double[]> saveAll() {
            List<double[]> s = new ArrayList<>();
            for (var b : blocks) s.add(b.save());
            return s;
        }

        void restoreAll(List<double[]> s) {
            for (int i = 0; i < blocks.size(); i++) blocks.get(i).restore(s.get(i));
        }

        DMatrixRMaj lastH;
        double lastCost;

        int optimize() {
            DMatrixRMaj H = new DMatrixRMaj(numParams, numParams);
            DMatrixRMaj g = new DMatrixRMaj(numParams, 1);
            DMatrixRMaj A = new DMatrixRMaj(numParams, numParams);
            DMatrixRMaj delta = new DMatrixRMaj(numParams, 1);
            LinearSolverDense<DMatrixRMaj> chol = LinearSolverFactory_DDRM.chol(numParams);

            double lambda = 1e-3;
            double cost = linearize(H, g);
            int iter = 0;
            for (; iter < settings.maxIterations(); iter++) {
                boolean stepped = false;
                for (int attempt = 0; attempt < 10; attempt++) {
                    A.setTo(H);
                    for (int i = 0; i < numParams; i++) {
                        A.add(i, i, lambda * H.get(i, i) + 1e-9);
                    }
                    if (!chol.setA(A)) {
                        lambda *= 10;
                        continue;
                    }
                    chol.solve(g, delta);
                    CommonOps_DDRM.scale(-1, delta);

                    var saved = saveAll();
                    double[] d = delta.getData();
                    for (var b : blocks) b.apply(d);
                    double newCost = totalCost();
                    if (Double.isFinite(newCost) && newCost < cost) {
                        double improvement = (cost - newCost) / Math.max(cost, 1e-12);
                        cost = newCost;
                        lambda = Math.max(lambda / 3, 1e-9);
                        stepped = true;
                        if (improvement < 1e-10) {
                            iter = settings.maxIterations();
                        }
                        break;
                    } else {
                        restoreAll(saved);
                        lambda *= 10;
                    }
                }
                if (!stepped) break;
                cost = linearize(H, g);
                if (iter >= settings.maxIterations()) break;
            }
            lastCost = linearize(H, g);
            lastH = H;
            return Math.min(iter + 1, settings.maxIterations());
        }

        // -------------------------------- results --------------------------------

        ExtrinsicSolveResult buildResult(int iterations) {
            int residualCount = 0;
            for (var f : factors) residualCount += f.residualDim();
            double dof = Math.max(1, residualCount - numParams);
            // Inflate (never shrink) the covariance by the reduced chi-squared. Shrinking would also
            // shrink the odometry-driven uncertainty when corners are merely cleaner than assumed.
            double varianceFactor = Math.max(lastCost / dof, 1.0);

            DMatrixRMaj cov = new DMatrixRMaj(numParams, numParams);
            DMatrixRMaj Hreg = lastH.copy();
            double maxDiag = 0;
            for (int i = 0; i < numParams; i++) maxDiag = Math.max(maxDiag, Hreg.get(i, i));
            for (int i = 0; i < numParams; i++) Hreg.add(i, i, 1e-12 * maxDiag + 1e-12);
            boolean haveCov = CommonOps_DDRM.invert(Hreg, cov);

            double sumSq = 0;
            int cornerCount = 0;
            List<ExtrinsicSolveResult.CameraResult> camResults = new ArrayList<>();
            double[] uv = new double[2];
            double[] pw = new double[3];
            double[] pc = new double[3];
            Map<Cam, double[]> perCam = new HashMap<>();
            for (var o : observations) {
                Rigid3 worldToCam = o.stop.pose.pose().compose(o.cam.X.T).inverse();
                Rigid3 tagPose = o.tag.pose();
                double[] acc = perCam.computeIfAbsent(o.cam, k -> new double[2]);
                for (int i = 0; i < 4; i++) {
                    tagPose.transformPoint(tagCorners[i], pw);
                    worldToCam.transformPoint(pw, pc);
                    double e2;
                    if (project(o.cam.K, o.cam.dist, pc, uv)) {
                        double dx = uv[0] - o.uv[2 * i];
                        double dy = uv[1] - o.uv[2 * i + 1];
                        e2 = dx * dx + dy * dy;
                    } else {
                        e2 = 1e6;
                    }
                    acc[0] += e2;
                    acc[1] += 1;
                    sumSq += e2;
                    cornerCount++;
                }
            }

            boolean allGood = true;
            for (var cam : cams.values()) {
                List<String> camWarnings = new ArrayList<>();
                double[] std = new double[6];
                for (int i = 0; i < 6; i++) {
                    double v =
                            haveCov
                                    ? cov.get(cam.X.offset + i, cam.X.offset + i) * varianceFactor
                                    : Double.POSITIVE_INFINITY;
                    std[i] = Math.sqrt(Math.max(v, 0));
                    if (i >= 3) std[i] = Math.toDegrees(std[i]);
                }
                if (std[0] > WARN_TRANSLATION_STDDEV_M || std[1] > WARN_TRANSLATION_STDDEV_M) {
                    camWarnings.add(
                            "X/Y poorly constrained. Rotate the robot in place to more headings while this camera sees tags.");
                }
                if (std[2] > WARN_TRANSLATION_STDDEV_M) {
                    camWarnings.add(
                            "Height poorly constrained. Make sure this camera sees a floor tag or a tag at a known height.");
                }
                if (std[3] > WARN_ROTATION_STDDEV_DEG || std[4] > WARN_ROTATION_STDDEV_DEG) {
                    camWarnings.add("Roll/pitch poorly constrained. Capture more stops with tags in view.");
                }
                if (std[5] > WARN_ROTATION_STDDEV_DEG) {
                    camWarnings.add(
                            "Yaw poorly constrained. Drive the robot in straight lines (forward/back and sideways) between stops.");
                }
                if (!camWarnings.isEmpty()) allGood = false;

                double[] acc = perCam.getOrDefault(cam, new double[] {0, 1});
                Set<StopState> stopsSeen = new HashSet<>();
                for (var o : cam.obs) stopsSeen.add(o.stop);
                Pose3d pose = cam.X.T.toPose();
                var rot = pose.getRotation();
                camResults.add(
                        new ExtrinsicSolveResult.CameraResult(
                                cam.info.uniqueName(),
                                cam.info.nickname(),
                                pose.getX(),
                                pose.getY(),
                                pose.getZ(),
                                Math.toDegrees(rot.getX()),
                                Math.toDegrees(rot.getY()),
                                Math.toDegrees(rot.getZ()),
                                std,
                                Math.sqrt(acc[0] / acc[1]),
                                cam.obs.size(),
                                stopsSeen.size(),
                                camWarnings));
            }

            List<ExtrinsicSolveResult.TagResult> tagResults = new ArrayList<>();
            for (var tag : tags.values()) {
                Pose3d p = tag.pose().toPose();
                var rot = p.getRotation();
                tagResults.add(
                        new ExtrinsicSolveResult.TagResult(
                                tag.id,
                                p.getX(),
                                p.getY(),
                                p.getZ(),
                                Math.toDegrees(rot.getX()),
                                Math.toDegrees(rot.getY()),
                                Math.toDegrees(rot.getZ()),
                                tag.floor,
                                tag.fixed,
                                tag.observations));
            }

            double rms = Math.sqrt(sumSq / Math.max(cornerCount, 1));
            if (rms > 2.0) {
                warnings.add(
                        String.format(
                                "High reprojection error (%.2f px). Check tag size, floor tag IDs, intrinsic calibration, and that no tag ID is used twice.",
                                rms));
            }
            if (settings.estimateOdometryScale() && Math.abs(scale.v - 1) > 0.03) {
                warnings.add(
                        String.format(
                                "Odometry and tags disagree on distances by %.1f%%. Check the tag size or your wheel radius.",
                                (scale.v - 1) * 100));
            }
            String message =
                    allGood && warnings.isEmpty()
                            ? String.format("Solved %d camera(s), RMS error %.2f px", camResults.size(), rms)
                            : String.format(
                                    "Solved %d camera(s) with warnings, RMS error %.2f px", camResults.size(), rms);
            logger.info(message);
            return new ExtrinsicSolveResult(
                    true, message, camResults, tagResults, scale.v, rms, iterations, stops.size(), warnings);
        }

        static double median(List<Double> values) {
            double[] arr = values.stream().mapToDouble(Double::doubleValue).sorted().toArray();
            int n = arr.length;
            return n % 2 == 1 ? arr[n / 2] : (arr[n / 2 - 1] + arr[n / 2]) / 2;
        }
    }
}
