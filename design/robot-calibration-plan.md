# Robot Camera Calibration: Intrinsics + Robot-Frame Extrinsics

Status: proposal / investigation. Nothing here is implemented yet.

## 1. Summary

**Goal:** stop relying on CAD for `robotToCamera`. PhotonVision should measure the intrinsics *and* the
robot-to-camera transform of every camera attached to an instance, as-built, in a short guided procedure.

**Recommendation (short version):**

1. **Do not make Kalibr + a coprocessor-attached CANivore/Pigeon the primary path.** Camera-IMU
   calibration answers "where is the camera relative to the IMU", not "where is the camera relative to
   the robot", and it needs the sensor rig to be waved through aggressive 6-DoF motion — not something
   you can do with a 125 lb robot. Details in [section 3](#3-why-not-kalibr--canivore-as-the-primary-path).
2. **Instead, calibrate directly against the robot frame using the drivetrain.** The robot frame in
   team code *is* defined by the drivetrain kinematics (module positions around the center) and the
   floor. So: put a handful of AprilTags around a practice area, have the robot drive a short
   stop-and-go routine (spin + translate), and jointly solve for every camera's `robotToCamera` using
   (a) tag detections from all cameras, (b) wheel odometry + Pigeon 2 yaw streamed from the roboRIO
   over NetworkTables, and (c) the floor as the height datum. This is a planar hand-eye / factor-graph
   problem, and it handles non-overlapping cameras naturally.
3. **The Pigeon 2 stays where it already is (on the roboRIO's CAN bus).** No CANivore on the coprocessor,
   no Phoenix on the coprocessor, no IMU noise characterization.
4. **No hardware camera sync is needed.** The capture is stop-and-go: frames are only used while the
   robot is stationary, which removes the timing problem entirely (and reuses the stability gating
   that auto-calibration already has).
5. **Improve intrinsics** with coverage/uncertainty-driven guidance, lens model choice, and a
   "calibrate all cameras" flow, building on the auto-capture work that just landed (#2645, #2653).
6. **Close the loop at runtime:** store `robotToCamera` on the coprocessor, publish it over NT, let
   PhotonLib consume it, and monitor for mount drift (a bumped camera) during matches.

Kalibr/Basalt remain useful as an *offline cross-check*: the dataset recorder can export in their
formats as a later, optional phase.

## 2. What exists today

All paths relative to repo root.

| Area | Current state |
| --- | --- |
| Board detection | `photon-core/.../vision/pipe/impl/FindBoardCornersPipe.java` — OpenCV ChArUco (default) or chessboard. |
| Intrinsic solver | `photon-core/.../vision/pipe/impl/Calibrate3dPipe.java` — mrcal via `MrCalJNI` (8-coef OpenCV model, board warp) with OpenCV `calibrateCameraExtended` fallback. |
| Capture UX | `photon-core/.../vision/pipeline/Calibrate3dPipeline.java` — manual snapshots + new auto-capture (moved ≥5% of diagonal, stable for 3 frames). |
| Stored data | `CameraCalibrationCoefficients` (intrinsics, distortion, per-snapshot `BoardObservation`s, warp, `rt_cam_ref` poses); `estimateUncertainty()` exists. Stored per resolution in `CameraConfiguration.calibrations`, JSON in `photon.sqlite`. |
| Lifecycle / API | `VisionModule.startCalibration/takeCalibrationSnapshot/computeCalibration/commitCalibration`; REST under `/api/calibration/*` (`Server.java`, `RequestHandler.java`). |
| UI | `photon-client/src/components/cameras/CameraCalibrationCard.vue`, `CameraCalibrationInfoCard.vue`, calibration + uncertainty visualizers. |
| robotToCamera | **Not stored anywhere on the coprocessor.** Exists only in user robot code (`PhotonPoseEstimator`, `VisionEstimation.estimateRobotPoseConstrainedSolvepnp`). |
| Timestamps | USB: cscore grab time; CSI: libcamera capture time mapped to NT time. `TimeSyncManager` aligns coprocessor and roboRIO clocks. |
| IMU / CAN | Nothing. No CTRE/Phoenix dependency anywhere. |
| Multi-camera | Each `VisionModule` runs independently on its own `VisionRunner` thread; no cross-camera coordination. |
| Native deps | mrcal JNI for all platforms; libcamera / rknn / tflite only on linuxarm64. Constrained solvePnP uses CasADi-generated code in `photon-targeting`. |

## 3. Why not Kalibr + CANivore as the primary path

I looked at this seriously because it is the "textbook" answer. It is a poor fit for an FRC robot:

1. **It calibrates to the wrong frame.** Kalibr's camera-IMU mode outputs `T_cam_imu`. To get
   `robotToCamera` you still need `T_robot_imu` — i.e. where the Pigeon is mounted, which is the same
   "students mounted it, CAD is approximate" problem, just moved to a different part.
2. **The required motion is impossible on a full robot.** Camera-IMU calibration needs strong
   rotation about all three axes and acceleration in all directions. With rotation about one axis only
   (a robot spinning on the floor), the extrinsic translation along that axis is provably unobservable
   (Yang, Geneva, Eckenhoff, Huang, *Degenerate Motion Analysis for Aided INS*, RA-L 2019). You would
   have to calibrate the camera mount off the robot and then re-attach it — which reintroduces exactly
   the error we're trying to remove.
3. **Toolchain cost.** Kalibr is ROS1-only (catkin, Docker images up to Ubuntu 20.04), effectively
   unmaintained since 2022, CPU/RAM heavy, and needs an IMU noise model from a multi-hour static
   Allan-variance recording (CTRE publishes no noise density for the Pigeon 2). It would have to run
   offline on a laptop, not on the coprocessor.
4. **Phoenix 6 on the coprocessor is a risk.** CTRE's non-FRC Linux support lists C++/C#/Python
   only (no Java), requires the `canivore-usb` kernel module (documented for Raspberry Pi and Jetson,
   not Orange Pi vendor kernels), and a CANivore is owned by a single USB host — on most robots it
   already belongs to the roboRIO. CANivore-timestamped signals work without Pro; device timestamps
   and timesync require Phoenix Pro.
5. **Non-overlapping cameras.** Kalibr's multi-camera mode needs overlapping views between cameras; the
   workaround is per-camera-to-IMU then chaining, which compounds error. Robot cameras rarely overlap.

What the IMU *is* good for here: precise relative yaw between stops, and the gravity vector for
robot pitch/roll on an uneven floor. Both are available from the Pigeon that's already on the robot.

## 4. Proposed approach: drive-to-calibrate robot extrinsics

### 4.1 Key insight

`robotToCamera` is only meaningful relative to the frame your pose estimator uses. That frame is
defined by:

- **x, y, yaw:** the drivetrain kinematics (module positions around the robot center, module "forward").
  Wheel odometry + gyro measure motion *in exactly this frame*.
- **z, roll, pitch:** the floor the robot sits on.

So we calibrate against those two things directly. Neither CAD nor the Pigeon's mounting position is involved.

### 4.2 Physical setup ("tag kit")

- 6–12 AprilTags (36h11, size printed and verified with calipers — tag size sets metric scale) on rigid
  boards (foam board / plywood) placed around a ~3×3 m area, at varied heights and orientations, so
  that every camera sees at least two tags from most robot headings.
- **At least one tag lying flat on the floor.** This is the height datum (z, roll, pitch). Alternatively
  a tag at a tape-measured height.
- The tag layout does **not** need to be measured: the solver co-estimates tag poses (same idea as
  WPIcal's field mapping). Optionally, load a known layout (real field JSON, or a WPIcal output) to
  skip map estimation.

### 4.3 Robot data (roboRIO → coprocessor)

A small PhotonLib helper (Java/C++/Python parity) publishes over NT:

- odometry-only pose (`Pose2d`; **must not** be vision-fused) and its FPGA timestamp,
- Pigeon yaw (and optionally gravity vector / pitch / roll),
- chassis speeds or an explicit "stationary" flag.

`TimeSyncManager` already maps roboRIO time to coprocessor time; with stop-and-go capture the required
precision is loose (~tens of ms).

Optionally the helper also provides a command that drives the calibration routine automatically
(given a `Consumer<ChassisSpeeds>`): rotate in ~30° steps through 360°, translate forward/back/left/right
~0.5–1 m, repeat at a couple of spots, pausing ~0.5 s at each stop.

### 4.4 Capture (coprocessor)

A new cross-module **extrinsic calibration session** (the first thing that coordinates multiple
`VisionModule`s):

1. Precondition: every participating camera has an intrinsic calibration at the selected resolution.
2. Each module runs AprilTag detection at full resolution (no decimation) and buffers detections.
3. When odometry reports stationary **and** each camera's detections are stable for N frames (reuse
   the stability logic from `Calibrate3dPipeline`), record a *stop*: averaged/median tag corners per
   camera, the odometry pose, Pigeon yaw/gravity, and a downscaled thumbnail for review.
4. Live UI feedback: number of stops, robot headings covered, per-camera tag coverage, and an
   observability meter from the solver's information matrix (see 4.6).
5. Persist the whole session as a self-contained dataset (JSON + images in a zip) so it can be re-solved
   later, attached to bug reports, used as a regression test fixture, and exported to other tools.

### 4.5 Solver (factor graph)

Unknowns:

- `X_c` ∈ SE(3): robot → camera *c*, for every camera (what we want),
- `R_k` ∈ SE(2) (embedded in SE(3) on the floor plane): robot pose in the tag world at stop *k*,
- `T_j` ∈ SE(3): pose of tag *j* (unless a known layout is supplied),
- nuisance: odometry scale factor (wheel radius error), optional per-camera time offset (only if
  non-stationary frames are ever used).

Factors:

- **Tag corner reprojection:** `π(K_c, d_c, (R_k · X_c)⁻¹ · T_j · corner)` vs detected pixel, with
  the camera's mrcal intrinsics held fixed (or loosely priored), and a robust (Huber/Cauchy) loss.
- **Odometry between-factors:** `R_k⁻¹ R_{k+1}` vs odometry delta, with yaw from the Pigeon (tight)
  and translation from wheels (looser, scale-corrected).
- **Floor / gravity:** robot poses constrained to the world floor plane (z = 0, roll = pitch = 0), or
  softly priored from Pigeon pitch/roll if the floor isn't flat.
- **Datum:** floor tag(s) define the world z = 0 plane; gauge fixed by anchoring one tag (or the first
  robot pose).

Observability (this is why each ingredient is needed):

| `robotToCamera` component | Determined by |
| --- | --- |
| z, roll, pitch | Floor datum: camera pose in the world vs the floor plane the robot sits on. |
| x, y | Robot rotations in place: the camera traces a circle around the kinematic center. |
| yaw | Robot translations: the direction odometry calls "+x" as seen in the world. Pure spinning cannot fix yaw. |
| metric scale | Tag size (primary), odometry (secondary/nuisance). |

Without odometry, robot x/y/yaw inside its own frame is a free gauge — any camera-only method
(including multi-camera mrcal/Kalibr) can only give camera-to-camera transforms, not robot-to-camera.

Outputs per camera: `robotToCamera`, its 6×6 covariance (from the information matrix), RMS reprojection
error, per-stop residuals, and the estimated tag map. Because all cameras share `R_k`, the solve also
yields consistent camera-to-camera transforms even with no overlapping views.

Library choice:

- **GTSAM (preferred)** — BSD, built exactly for this (Pose2/Pose3, BetweenFactor, projection factors,
  robust noise models, marginal covariances). Ship as a JNI artifact per platform following the
  `mrcal-java` pattern. Prototype first with GTSAM's Python wheels against recorded datasets.
- **Ceres** — BSD alternative if GTSAM's ARM build/size is a problem; covariance via `ceres::Covariance`.
- mrcal itself can't take odometry factors, so it stays the intrinsics solver.

Expected accuracy: to be measured in phase 0, not promised. Target: ≤ 5 mm translation and ≤ 0.3°
rotation (1σ) per camera, which is better than typical 3D-printed-mount tolerance.

### 4.6 Guidance and validation in the UI

- Observability meter: smallest eigenvalues of the marginal information for each `X_c`, mapped to
  "need more rotation" / "need more translation" / "camera *c* sees too few tags".
- Review screen: 3D view of the robot, cameras (CAD value vs calibrated, if the user enters CAD), tag
  map and stop poses; residual histogram; per-camera ±σ.
- Hold-out check: re-solve with 20% of stops held out; report reprojection error on held-out stops.

### 4.7 Fixture mode (fallback, same solver)

For robots that can't drive (on a cart, early in build season): push the robot into a square corner
jig with tags at known positions relative to the corner. The jig supplies a tight prior on `R_k`
instead of odometry. Less accurate (bumper compliance), but uses the identical solver and UI.

## 5. Intrinsic calibration improvements

Building on auto-capture (#2645) and preliminary info (#2653):

1. **Coverage + tilt guidance:** overlay a heatmap of where corners have been observed, plus a tilt
   diversity indicator. mrcal's own guidance is that boards close to the lens, tilted, and filling the
   corners matter most.
2. **Uncertainty-driven stop condition:** compute mrcal projection uncertainty
   (`estimateUncertainty()` already exists) incrementally and tell the user when it's below a
   threshold across the image ("done") or where it's still high ("cover the top-left corner").
3. **Blur rejection:** sharpness check (variance of Laplacian over the board ROI) in addition to the
   existing centroid stability check.
4. **Outlier handling:** show mrcal's outlier rejection results and per-snapshot residuals; one-click
   drop of the worst snapshots and re-solve without re-capturing.
5. **Lens model selection:** expose `useMrCal` and lens models (OpenCV8 vs splined stereographic for
   wide-FOV OV9281 lenses), defaulting sensibly.
6. **Repeatability check:** compare two calibrations of the same camera (mrcal projection diff) and
   report the difference in pixels — the honest measure of calibration quality.
7. **"Calibrate all cameras" flow:** one wizard that walks through every camera on the instance, as a
   precondition for extrinsics.
8. **Docs:** lock the M12 lens focus (thread locker) *before* calibrating; refocusing invalidates it.

## 6. Runtime integration

1. **Storage:** add `robotToCamera` (`Transform3d`) plus metadata (method, date, covariance, resolution,
   dataset id) to `CameraConfiguration`. It's JSON in SQLite, so the field is additive. Check whether
   `DbMigration` needs a version bump.
2. **Publish:** NT topic per camera, e.g. `photonvision/<camera>/robotToCamera` (Transform3d struct)
   + uncertainty.
3. **PhotonLib:** `PhotonCamera.getRobotToCamera()` (Optional), and a `PhotonPoseEstimator` option to
   use the camera-provided value instead of a hardcoded one, with a warning if code and coprocessor disagree.
4. **Manual entry:** UI field for teams that still want to type CAD values. This is useful on its own
   and lands early.
5. **Drift monitor (later):** during normal operation, compare each camera's single-camera robot pose
   against the fused/other-camera estimate; a persistent bias flags a bumped mount ("camera *front-left*
   appears rotated ~2° since last calibration").
6. **Coprocessor-side robot pose:** with extrinsics on the coprocessor, multi-camera MultiTag →
   robot pose becomes possible as a later feature.

## 7. Hardware recommendations

| Item | Recommendation |
| --- | --- |
| IMU | Use the robot's existing Pigeon 2 via the roboRIO; stream yaw (+gravity) over NT. **No CANivore on the coprocessor.** |
| Camera sync | **Not required.** Stop-and-go capture makes it irrelevant for calibration. Hardware triggering (OV9281 USB boards have an external trigger input; MIPI variants have XVS/FSIN) could help *runtime* multi-camera fusion later; evaluate separately. |
| Tags | 6–12 printed 36h11 tags on rigid flat boards, size verified with calipers; ≥1 flat on the floor. |
| Floor | Reasonably flat area (carpet is fine); Pigeon gravity can absorb mild tilt. |

If you later want a coprocessor-attached IMU for something like visual-inertial odometry, that's a separate
project with the Phoenix-on-ARM risks listed in section 3.

## 8. Implementation plan

### Phase 0 — De-risk the math (offline, small)

- Synthetic data generator using PhotonLib simulation (`VisionSystemSim`, `SimCameraProperties`):
  known `X_c`, tag map, motion script, noise → detections + odometry.
- Python GTSAM prototype of the solver; verify recovery, verify predicted degeneracies (spin-only ⇒ yaw
  unobservable; no floor tag ⇒ z unobservable).
- Record a real dataset with a quick hack (AprilTag pipeline corners + NT odometry log) and measure
  repeatability over ~5 runs and against a carefully measured camera.
- **Exit criterion:** repeatability within the 5 mm / 0.3° target on a real robot. If not, revisit
  before building UI.

### Phase 1 — Intrinsics improvements (medium, independent)

Items from section 5, in `Calibrate3dPipeline`, `Calibrate3dPipe`, `CameraCalibrationCard.vue`.

### Phase 2 — `robotToCamera` plumbing (small–medium)

- `CameraConfiguration` field + REST endpoints + manual-entry UI.
- NT publishing in `NTDataPublisher`.
- PhotonLib Java/C++/Python: `getRobotToCamera()`, `PhotonPoseEstimator` integration, docs.

### Phase 3 — Capture session + dataset (medium)

- `photon-core/.../vision/calibration/extrinsic/`: `ExtrinsicCalibrationSession` (multi-module
  coordinator), `RobotStateSubscriber` (NT odometry/gyro, time-synced via `TimeSyncManager`),
  `StationaryDetector`, `ExtrinsicDataset` (serialize/zip).
- PhotonLib helper: `ExtrinsicCalibrationPublisher` + optional auto-routine command.
- REST: `/api/extrinsics/{start,stop,status,solve,commit,export}`; websocket progress events.
- UI: a "Robot Calibration" wizard (preconditions → tag kit → robot link status → capture → review → commit).

### Phase 4 — On-device solver (large)

- GTSAM (or Ceres) JNI artifact built per platform (linuxarm64, linuxx86-64, windows, macOS) like
  `mrcal-java`; `ExtrinsicSolverJNI` loaded via `LoadJNI`.
- Observability meter, covariance, hold-out validation, 3D review view.
- Regression tests: synthetic datasets + one recorded real dataset in `test-resources`.

### Phase 5 — Optional extensions

- Drift monitor at runtime.
- Export dataset to Kalibr/Basalt format for cross-checking.
- Fixture mode (4.7).
- Hardware camera sync investigation for runtime fusion.

## 9. Risks / open questions

1. **Odometry quality.** Swerve slip during spins/translations. Mitigation: slow motions, robust loss,
   scale nuisance, tight Pigeon yaw. Phase 0 measures it.
2. **GTSAM on ARM.** Build size/time and dependencies for the JNI artifact; Ceres is the fallback.
3. **Robot frame convention.** Some teams put the origin at the frame center vs kinematic center, or at
   floor vs bumper height. Expose an explicit robot-origin offset and document the convention
   (default: kinematic center, on the floor).
4. **Cameras on mechanisms** (turrets, elevators) are out of scope for v1. They'd need the mechanism
   state as another factor.
5. **Multiple coprocessors.** Each instance calibrates its own cameras. A shared tag map / dataset
   between instances would give one consistent solve and is a possible follow-up.
6. **Tag size accuracy** directly scales translation. Measure printed tags; the solver should report
   a tag-vs-odometry scale disagreement.

## 10. References

- Kalibr: <https://github.com/ethz-asl/kalibr> (camera-IMU wiki, IMU noise model wiki)
- mrcal: <https://mrcal.secretsauce.net/>
- Basalt calibration: <https://gitlab.com/VladyslavUsenko/basalt/-/blob/master/doc/Calibration.md>
- MC-Calib (non-overlapping multi-camera): <https://github.com/rameau-fr/MC-Calib>
- Yang et al., degenerate motion for aided INS (RA-L 2019): <https://udel.edu/~pgeneva/downloads/papers/r05.pdf>
- Guo, Mirzaei, Roumeliotis, odometer-camera extrinsics (ICRA 2012)
- Heng et al., CamOdoCal (IROS 2013)
- Phoenix 6 non-FRC install / requirements: <https://v6.docs.ctr-electronics.com/en/stable/docs/installation/installation-nonfrc.html>
- Pigeon 2 user guide: <https://ctre.download/files/user-manual/Pigeon2%20User's%20Guide.pdf>
- Arducam external trigger docs: <https://docs.arducam.com/USB-Industrial-Camera/Connect-and-Sync-Multi-Stereo-Cameras/Hardware-Trigger-with-External-Signal/>
