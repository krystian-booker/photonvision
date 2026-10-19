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

import io.avaje.jsonb.Json;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;
import java.util.function.Consumer;
import java.util.function.LongSupplier;
import org.jetbrains.annotations.Nullable;
import org.photonvision.common.logging.LogGroup;
import org.photonvision.common.logging.Logger;
import org.photonvision.vision.pipeline.result.CVPipelineResult;
import org.wpilib.math.geometry.Pose3d;

/**
 * Runs a robot extrinsic calibration session across every camera on this coprocessor.
 *
 * <p>While capturing, the robot drives a stop-and-go routine. Each time odometry shows the robot
 * has settled, the AprilTag detections of every participating camera are collected for a few frames
 * and combined into one {@link ExtrinsicDataset.Stop}. Only frames taken while the robot is at rest
 * are used, so no camera/IMU time synchronization (or hardware camera sync) is needed.
 *
 * <p>Cameras must be running an AprilTag pipeline and have an intrinsic calibration at their
 * current resolution.
 */
public class ExtrinsicCalibrationManager {
    private static final Logger logger =
            new Logger(ExtrinsicCalibrationManager.class, LogGroup.General);

    // The robot counts as still if odometry moved less than this over STILL_WINDOW
    private static final double STILL_TRANSLATION_M = 0.003;
    private static final double STILL_ROTATION_RAD = Math.toRadians(0.3);
    private static final long STILL_WINDOW_MICROS = 250_000;
    // Odometry older than this means the robot isn't publishing
    private static final long ODOMETRY_TIMEOUT_MICROS = 500_000;
    // Give up waiting for slow/absent cameras after this long
    private static final long COLLECT_TIMEOUT_MICROS = 2_000_000;
    // A camera with no results for this long is considered absent for the current stop
    private static final long CAMERA_ACTIVE_MICROS = 1_000_000;
    private static final long BROADCAST_PERIOD_MICROS = 250_000;
    private static final long HISTORY_MICROS = 3_000_000;

    public enum Phase {
        /** Not capturing. */
        IDLE,
        /** Waiting for the robot to come to rest. */
        WAITING_FOR_STILL,
        /** Robot at rest, gathering frames from every camera. */
        COLLECTING,
        /** Stop recorded; waiting for the robot to move far enough for the next one. */
        WAITING_FOR_MOTION
    }

    /**
     * Capture settings.
     *
     * @param cameras Unique names of the cameras to calibrate
     * @param tagSizeMeters Edge length of the tags' black square
     * @param minTranslationMeters Robot travel required between stops
     * @param minRotationDegrees Robot rotation required between stops (either suffices)
     * @param settleSeconds How long the robot must be still before collecting
     * @param framesPerStop Frames to combine per camera at each stop
     */
    @Json
    public record CaptureSettings(
            List<String> cameras,
            double tagSizeMeters,
            double minTranslationMeters,
            double minRotationDegrees,
            double settleSeconds,
            int framesPerStop) {
        public static CaptureSettings defaults(List<String> cameras) {
            return new CaptureSettings(cameras, 0.1651, 0.15, 15, 0.5, 5);
        }
    }

    /**
     * Live state of one camera during a session.
     *
     * @param uniqueName Camera unique name
     * @param nickname Camera nickname
     * @param hasCalibration Whether an intrinsic calibration exists at the current resolution
     * @param resolution Current resolution, e.g. "1280x800"
     * @param tagsVisible Tags in the most recent frame
     * @param observations Tag observations recorded so far
     * @param stopsWithTags Stops at which this camera saw at least one tag
     */
    @Json
    public record CameraStatus(
            String uniqueName,
            String nickname,
            boolean hasCalibration,
            String resolution,
            int tagsVisible,
            int observations,
            int stopsWithTags) {}

    /**
     * Everything the UI needs to show.
     *
     * @param phase Current {@link Phase}
     * @param odometryConnected Whether odometry is arriving from the robot
     * @param robotStill Whether the robot is currently at rest
     * @param odometry Latest odometry [x, y, thetaDegrees], or empty
     * @param stopCount Stops recorded
     * @param headingsCovered Distinct 30 degree heading buckets covered by recorded stops (of 12)
     * @param settings Current capture settings
     * @param cameras Per-camera status
     * @param message Latest human readable event
     * @param result Most recent solve result, or null
     */
    @Json
    public record Status(
            String phase,
            boolean odometryConnected,
            boolean robotStill,
            double[] odometry,
            int stopCount,
            int headingsCovered,
            CaptureSettings settings,
            List<CameraStatus> cameras,
            String message,
            @Nullable ExtrinsicSolveResult result) {}

    private static final class CameraState {
        String nickname;
        @Nullable ExtrinsicDataset.CameraInfo info;
        String resolution = "";
        int tagsVisible = 0;
        long lastResultMicros = Long.MIN_VALUE;
        final List<Map<Integer, double[]>> frames = new ArrayList<>();
    }

    private static ExtrinsicCalibrationManager instance;

    public static synchronized ExtrinsicCalibrationManager getInstance() {
        if (instance == null) {
            instance =
                    new ExtrinsicCalibrationManager(
                            new RobotOdometrySource.NetworkTablesSource(),
                            org.wpilib.networktables.NetworkTablesJNI::now,
                            ExtrinsicCalibrationBroadcaster::broadcast);
        }
        return instance;
    }

    private final RobotOdometrySource odometrySource;
    private final LongSupplier clockMicros;
    private final Consumer<Status> broadcaster;

    private CaptureSettings settings = CaptureSettings.defaults(List.of());
    // Volatile so camera threads can skip the lock when no session is running
    private volatile Phase phase = Phase.IDLE;
    private final ArrayDeque<RobotOdometrySource.Sample> history = new ArrayDeque<>();
    private long stillSinceMicros = -1;
    private long collectStartMicros = 0;
    private final List<RobotOdometrySource.Sample> collectOdometry = new ArrayList<>();
    private final Map<String, CameraState> cameras = new LinkedHashMap<>();
    private final List<ExtrinsicDataset.Stop> stops = new ArrayList<>();
    private double tagSizeMeters = settings.tagSizeMeters();
    private RobotOdometrySource.Sample lastStopPose = null;
    private ExtrinsicSolveResult lastResult = null;
    private String message = "";
    private long lastBroadcastMicros = Long.MIN_VALUE;
    private boolean captureRequested = false;

    ExtrinsicCalibrationManager(
            RobotOdometrySource odometrySource, LongSupplier clockMicros, Consumer<Status> broadcaster) {
        this.odometrySource = odometrySource;
        this.clockMicros = clockMicros;
        this.broadcaster = broadcaster;
    }

    // ------------------------------------------------------------------------------------------
    // Session control
    // ------------------------------------------------------------------------------------------

    /** Start (or resume) capturing. Existing stops are kept so a session can be paused. */
    public synchronized void start(CaptureSettings newSettings) {
        if (newSettings.cameras() == null || newSettings.cameras().isEmpty()) {
            throw new IllegalArgumentException("Select at least one camera");
        }
        if (newSettings.tagSizeMeters() <= 0) {
            throw new IllegalArgumentException("Tag size must be positive");
        }
        if (!stops.isEmpty() && Math.abs(newSettings.tagSizeMeters() - tagSizeMeters) > 1e-9) {
            throw new IllegalArgumentException(
                    "Tag size can't change mid-session. Reset the session first.");
        }
        settings = newSettings;
        tagSizeMeters = newSettings.tagSizeMeters();
        cameras.keySet().retainAll(newSettings.cameras());
        for (var name : newSettings.cameras()) cameras.computeIfAbsent(name, n -> new CameraState());
        phase = stops.isEmpty() ? Phase.WAITING_FOR_STILL : Phase.WAITING_FOR_MOTION;
        stillSinceMicros = -1;
        history.clear();
        odometrySource.poll(); // discard anything stale
        message = "Capturing. Drive to a spot and stop.";
        logger.info("Extrinsic calibration capture started for " + newSettings.cameras());
        broadcastNow();
    }

    public synchronized void stop() {
        phase = Phase.IDLE;
        message = "Capture paused with " + stops.size() + " stops";
        broadcastNow();
    }

    /** Discard all recorded stops and results. */
    public synchronized void reset() {
        phase = Phase.IDLE;
        stops.clear();
        lastStopPose = null;
        lastResult = null;
        for (var c : cameras.values()) c.frames.clear();
        message = "Session reset";
        broadcastNow();
    }

    /** Record a stop as soon as the robot is still, even if it hasn't moved far enough. */
    public synchronized void captureNow() {
        if (phase == Phase.IDLE) throw new IllegalStateException("Capture is not running");
        captureRequested = true;
        if (phase == Phase.WAITING_FOR_MOTION) phase = Phase.WAITING_FOR_STILL;
    }

    public synchronized void removeStop(int index) {
        if (index < 0 || index >= stops.size()) throw new IllegalArgumentException("No stop " + index);
        stops.remove(index);
        message = "Removed stop " + index;
        broadcastNow();
    }

    /** Replace the session with an imported dataset (for re-solving offline captures). */
    public synchronized void loadDataset(ExtrinsicDataset dataset) {
        if (dataset.stops() == null || dataset.cameras() == null) {
            throw new IllegalArgumentException("Dataset is missing stops or cameras");
        }
        phase = Phase.IDLE;
        stops.clear();
        stops.addAll(dataset.stops());
        cameras.clear();
        for (var info : dataset.cameras()) {
            var state = new CameraState();
            state.nickname = info.nickname();
            state.info = info;
            state.resolution = info.width() + "x" + info.height();
            cameras.put(info.uniqueName(), state);
        }
        tagSizeMeters = dataset.tagSizeMeters();
        settings =
                new CaptureSettings(
                        List.copyOf(cameras.keySet()),
                        tagSizeMeters,
                        settings.minTranslationMeters(),
                        settings.minRotationDegrees(),
                        settings.settleSeconds(),
                        settings.framesPerStop());
        lastResult = null;
        message = "Loaded dataset with " + stops.size() + " stops";
        broadcastNow();
    }

    public synchronized ExtrinsicDataset getDataset() {
        List<ExtrinsicDataset.CameraInfo> infos = new ArrayList<>();
        for (var e : cameras.entrySet()) {
            if (e.getValue().info != null) infos.add(e.getValue().info);
        }
        return new ExtrinsicDataset(
                ExtrinsicDataset.CURRENT_VERSION, tagSizeMeters, infos, List.copyOf(stops));
    }

    /**
     * Solve with the recorded stops.
     *
     * @param solverSettings Solver settings
     * @param layoutLookup Field layout tag poses, used when {@code solverSettings.useFieldLayout()}
     */
    public ExtrinsicSolveResult solve(
            ExtrinsicSolverSettings solverSettings,
            java.util.function.IntFunction<java.util.Optional<Pose3d>> layoutLookup) {
        ExtrinsicDataset dataset;
        synchronized (this) {
            dataset = getDataset();
        }
        Map<Integer, Pose3d> layout = new HashMap<>();
        if (solverSettings.useFieldLayout()) {
            for (var stop : dataset.stops()) {
                for (var o : stop.observations()) {
                    layoutLookup.apply(o.tagId()).ifPresent(p -> layout.put(o.tagId(), p));
                }
            }
        }
        // Solving can take a moment, so don't hold the lock (camera threads call in at frame rate)
        var result = ExtrinsicSolver.solve(dataset, solverSettings, layout);
        synchronized (this) {
            lastResult = result;
            message = result.message();
            broadcastNow();
        }
        return result;
    }

    public synchronized @Nullable ExtrinsicSolveResult getLastResult() {
        return lastResult;
    }

    // ------------------------------------------------------------------------------------------
    // Frame intake
    // ------------------------------------------------------------------------------------------

    /** Called with every pipeline result from every camera. Cheap when not capturing. */
    public void onPipelineResult(String uniqueName, String nickname, CVPipelineResult result) {
        if (phase == Phase.IDLE) return;
        synchronized (this) {
            if (phase == Phase.IDLE) return;
            long now = clockMicros.getAsLong();
            updateOdometry(now);

            var cam = cameras.get(uniqueName);
            if (cam != null) {
                cam.nickname = nickname;
                cam.lastResultMicros = now;
                updateCameraInfo(uniqueName, cam, result);

                Map<Integer, double[]> tags = new TreeMap<>();
                for (var t : result.targets) {
                    var corners = t.getTargetCorners();
                    if (t.getFiducialId() < 0 || corners == null || corners.size() != 4) continue;
                    double[] c = new double[8];
                    for (int i = 0; i < 4; i++) {
                        c[2 * i] = corners.get(i).x;
                        c[2 * i + 1] = corners.get(i).y;
                    }
                    tags.put(t.getFiducialId(), c);
                }
                cam.tagsVisible = tags.size();

                long frameMicros = result.getImageCaptureTimestampNanos() / 1000;
                if (frameMicros <= 0) frameMicros = now;
                if (phase == Phase.COLLECTING && frameMicros >= collectStartMicros && cam.info != null) {
                    cam.frames.add(tags);
                }
            }
            maybeFinishStop(now);
            maybeBroadcast(now);
        }
    }

    private void updateCameraInfo(String uniqueName, CameraState cam, CVPipelineResult result) {
        if (result.inputAndOutputFrame == null
                || result.inputAndOutputFrame.frameStaticProperties == null) {
            return;
        }
        var fsp = result.inputAndOutputFrame.frameStaticProperties;
        String resolution = fsp.imageWidth + "x" + fsp.imageHeight;
        var cal = fsp.cameraCalibration;
        if (cam.info != null && !resolution.equals(cam.resolution) && !stops.isEmpty()) {
            message =
                    cam.nickname
                            + " changed resolution mid-session; its new observations are ignored. Reset to use the new resolution.";
            cam.info = null;
            cam.resolution = resolution;
            return;
        }
        cam.resolution = resolution;
        if (cal == null) {
            cam.info = null;
            return;
        }
        if (cam.info == null) {
            cam.info =
                    new ExtrinsicDataset.CameraInfo(
                            uniqueName,
                            cam.nickname,
                            fsp.imageWidth,
                            fsp.imageHeight,
                            cal.getIntrinsicsArr(),
                            cal.getDistCoeffsArr());
        }
    }

    // ------------------------------------------------------------------------------------------
    // Motion state machine
    // ------------------------------------------------------------------------------------------

    private void updateOdometry(long now) {
        for (var s : odometrySource.poll()) {
            history.addLast(s);
            if (phase == Phase.COLLECTING) collectOdometry.add(s);
        }
        while (!history.isEmpty() && history.peekFirst().timestampMicros() < now - HISTORY_MICROS) {
            history.removeFirst();
        }

        boolean connected =
                !history.isEmpty() && now - history.peekLast().timestampMicros() < ODOMETRY_TIMEOUT_MICROS;
        boolean still = connected && isStill(now);

        if (!still) {
            stillSinceMicros = -1;
            if (phase == Phase.COLLECTING) {
                phase = Phase.WAITING_FOR_STILL;
                message =
                        connected
                                ? "Robot moved while collecting; waiting for it to stop again"
                                : "Lost odometry from the robot";
            }
            if (phase == Phase.WAITING_FOR_MOTION && connected && movedFarEnough()) {
                phase = Phase.WAITING_FOR_STILL;
            }
            return;
        }

        if (stillSinceMicros < 0) stillSinceMicros = now;
        if (phase == Phase.WAITING_FOR_MOTION && movedFarEnough()) {
            phase = Phase.WAITING_FOR_STILL;
        }
        if (phase == Phase.WAITING_FOR_STILL
                && (now - stillSinceMicros >= settings.settleSeconds() * 1e6 || captureRequested)) {
            if (lastStopPose == null || movedFarEnough() || captureRequested) {
                startCollecting(now);
            } else {
                phase = Phase.WAITING_FOR_MOTION;
            }
        }
    }

    private boolean isStill(long now) {
        var latest = history.peekLast();
        int n = 0;
        for (var it = history.descendingIterator(); it.hasNext(); ) {
            var s = it.next();
            if (s.timestampMicros() < now - STILL_WINDOW_MICROS) break;
            n++;
            if (Math.hypot(s.x() - latest.x(), s.y() - latest.y()) > STILL_TRANSLATION_M) return false;
            if (Math.abs(ExtrinsicSolver.wrapAngle(s.thetaRad() - latest.thetaRad()))
                    > STILL_ROTATION_RAD) {
                return false;
            }
        }
        // Robot code publishes every loop; require a couple of samples to call it still
        return n >= 2;
    }

    private boolean movedFarEnough() {
        if (lastStopPose == null) return true;
        var latest = history.peekLast();
        if (latest == null) return false;
        double dt = Math.hypot(latest.x() - lastStopPose.x(), latest.y() - lastStopPose.y());
        double dr = Math.abs(ExtrinsicSolver.wrapAngle(latest.thetaRad() - lastStopPose.thetaRad()));
        return dt >= settings.minTranslationMeters()
                || dr >= Math.toRadians(settings.minRotationDegrees());
    }

    private void startCollecting(long now) {
        phase = Phase.COLLECTING;
        captureRequested = false;
        collectStartMicros = now;
        collectOdometry.clear();
        collectOdometry.add(history.peekLast());
        for (var c : cameras.values()) c.frames.clear();
        message = "Robot stopped; collecting frames";
    }

    private void maybeFinishStop(long now) {
        if (phase != Phase.COLLECTING) return;
        boolean allDone = true;
        boolean anyFrames = false;
        for (var c : cameras.values()) {
            if (!c.frames.isEmpty()) anyFrames = true;
            boolean active = c.info != null && now - c.lastResultMicros < CAMERA_ACTIVE_MICROS;
            if (active && c.frames.size() < settings.framesPerStop()) allDone = false;
        }
        boolean timedOut = now - collectStartMicros > COLLECT_TIMEOUT_MICROS;
        if (!(allDone && anyFrames) && !timedOut) return;
        if (!anyFrames) {
            phase = Phase.WAITING_FOR_MOTION;
            lastStopPose = history.peekLast();
            message =
                    "No calibrated camera produced frames at this stop. Are the cameras running an AprilTag pipeline?";
            broadcastNow();
            return;
        }
        recordStop(now);
    }

    private void recordStop(long now) {
        double sx = 0, sy = 0, sc = 0, ss = 0;
        for (var s : collectOdometry) {
            sx += s.x();
            sy += s.y();
            sc += Math.cos(s.thetaRad());
            ss += Math.sin(s.thetaRad());
        }
        int n = collectOdometry.size();
        var pose = new RobotOdometrySource.Sample(now, sx / n, sy / n, Math.atan2(ss, sc));

        List<ExtrinsicDataset.TagObservation> observations = new ArrayList<>();
        int tagCount = 0;
        for (var e : cameras.entrySet()) {
            var frames = e.getValue().frames;
            if (frames.isEmpty()) continue;
            Map<Integer, List<double[]>> byTag = new TreeMap<>();
            for (var f : frames) {
                f.forEach((id, c) -> byTag.computeIfAbsent(id, k -> new ArrayList<>()).add(c));
            }
            int minFrames = Math.max(1, (frames.size() + 1) / 2);
            for (var t : byTag.entrySet()) {
                if (t.getValue().size() < minFrames) continue;
                observations.add(
                        new ExtrinsicDataset.TagObservation(
                                e.getKey(), t.getKey(), medianCorners(t.getValue()), t.getValue().size()));
                tagCount++;
            }
            frames.clear();
        }

        stops.add(
                new ExtrinsicDataset.Stop(
                        now / 1e6, pose.x(), pose.y(), pose.thetaRad(), List.copyOf(observations)));
        lastStopPose = pose;
        phase = Phase.WAITING_FOR_MOTION;
        message =
                String.format(
                        "Recorded stop %d (%d tag observations). Drive to the next spot.",
                        stops.size(), tagCount);
        logger.info(message);
        broadcastNow();
    }

    static double[] medianCorners(List<double[]> samples) {
        double[] out = new double[8];
        double[] column = new double[samples.size()];
        for (int i = 0; i < 8; i++) {
            for (int j = 0; j < samples.size(); j++) column[j] = samples.get(j)[i];
            java.util.Arrays.sort(column);
            int m = column.length;
            out[i] = m % 2 == 1 ? column[m / 2] : (column[m / 2 - 1] + column[m / 2]) / 2;
        }
        return out;
    }

    // ------------------------------------------------------------------------------------------
    // Status
    // ------------------------------------------------------------------------------------------

    public synchronized Status getStatus() {
        long now = clockMicros.getAsLong();
        if (phase == Phase.IDLE) {
            // Keep odometry fresh for the "robot connected" indicator even when not capturing
            for (var s : odometrySource.poll()) history.addLast(s);
            while (history.size() > 200) history.removeFirst();
        }
        var latest = history.peekLast();
        boolean connected = latest != null && now - latest.timestampMicros() < ODOMETRY_TIMEOUT_MICROS;

        boolean[] buckets = new boolean[12];
        for (var s : stops) {
            if (s.observations().isEmpty()) continue;
            double deg = Math.toDegrees(s.odomThetaRad());
            int b = (int) Math.floor(((deg % 360) + 360) % 360 / 30.0);
            buckets[Math.min(b, 11)] = true;
        }
        int covered = 0;
        for (boolean b : buckets) if (b) covered++;

        List<CameraStatus> camStatus = new ArrayList<>();
        for (var e : cameras.entrySet()) {
            int obs = 0;
            int stopsWithTags = 0;
            for (var s : stops) {
                int here = 0;
                for (var o : s.observations()) if (o.camera().equals(e.getKey())) here++;
                obs += here;
                if (here > 0) stopsWithTags++;
            }
            var c = e.getValue();
            camStatus.add(
                    new CameraStatus(
                            e.getKey(),
                            c.nickname == null ? e.getKey() : c.nickname,
                            c.info != null,
                            c.resolution,
                            c.tagsVisible,
                            obs,
                            stopsWithTags));
        }
        return new Status(
                phase.name(),
                connected,
                connected && phase != Phase.IDLE && stillSinceMicros >= 0,
                latest == null
                        ? new double[0]
                        : new double[] {latest.x(), latest.y(), Math.toDegrees(latest.thetaRad())},
                stops.size(),
                covered,
                settings,
                camStatus,
                message,
                lastResult);
    }

    public synchronized Phase getPhase() {
        return phase;
    }

    private void maybeBroadcast(long now) {
        if (now - lastBroadcastMicros >= BROADCAST_PERIOD_MICROS) broadcastNow();
    }

    private void broadcastNow() {
        lastBroadcastMicros = clockMicros.getAsLong();
        try {
            broadcaster.accept(getStatus());
        } catch (Exception e) {
            logger.error("Failed to broadcast extrinsic calibration status", e);
        }
    }
}
