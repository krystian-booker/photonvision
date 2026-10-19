# Calibrating Camera Mounting (Robot to Camera)

`PhotonPoseEstimator` needs to know where each camera sits on the robot: the robot-to-camera `Transform3d`. Teams usually copy this from CAD. However, 3D printed mounts, hand-assembled brackets and a few degrees of camera tilt add up. Even small errors here translate directly into robot pose error, and they get worse the farther away the tags are.

Robot calibration measures the robot-to-camera transform of **every camera on a PhotonVision instance at once**. You drive the robot around a few AprilTags while robot code streams its odometry to PhotonVision. No IMU attached to the coprocessor and no camera hardware sync are needed.

:::{important}
Each camera must be [intrinsically calibrated](calibration.md) at the resolution you will use, and must be running an AprilTag pipeline during robot calibration. Calibrate intrinsics first: robot calibration cannot fix bad intrinsics.
:::

## How it works

The robot frame your code uses is defined by your drivetrain and the floor:

- **x, y and yaw** come from the drivetrain. The origin is the center your kinematics are defined around, and +x is the direction your odometry calls forward.
- **z, roll and pitch** come from the floor the robot sits on.

So PhotonVision calibrates against exactly those two things:

- **Rotating in place** makes each camera trace a circle around the drivetrain center. That pins down each camera's **x and y**.
- **Driving in straight lines** shows which way is "forward" for odometry. That pins down each camera's **yaw**.
- **Tags at a known height** (a tag lying flat on the floor, a tag whose height you measured with a tape measure, or a known field layout) pin down each camera's **height, roll and pitch**.

Every time the robot comes to rest, PhotonVision combines a few frames of AprilTag corners from every camera into a *stop*. It then solves a single least-squares problem over all stops, all cameras and (optionally) all tag positions. Because only frames taken while the robot is still are used, camera and odometry timestamps don't need to be precisely synchronized.

The result for each camera includes a 1σ uncertainty, so you can tell whether the calibration is trustworthy. Cameras that never see the same tags still get consistent results, because they share the robot's motion.

## 1. Robot code

Publish your **odometry-only** pose every loop with `RobotCalibrationPublisher`:

```{eval-rst}
.. tab-set-code::

   .. code-block:: java

      private final RobotCalibrationPublisher calibrationPublisher = new RobotCalibrationPublisher();

      @Override
      public void robotPeriodic() {
          // Odometry only! Not a pose estimator that has vision measurements added.
          calibrationPublisher.update(odometry.getPose());
      }

   .. code-block:: c++

      photon::RobotCalibrationPublisher calibrationPublisher;

      void Robot::RobotPeriodic() {
        // Odometry only! Not a pose estimator that has vision measurements added.
        calibrationPublisher.Update(odometry.GetPose());
      }

   .. code-block:: python

      self.calibrationPublisher = RobotCalibrationPublisher()

      def robotPeriodic(self):
          # Odometry only! Not a pose estimator that has vision measurements added.
          self.calibrationPublisher.update(self.odometry.getPose())
```

:::{warning}
The pose **must not** include vision corrections. If you use a pose estimator, temporarily stop adding vision measurements, or keep a separate odometry object, for example a `SwerveDriveOdometry` fed with the same module positions and gyro angle.
:::

## 2. Set up tags

You can calibrate on a real field or anywhere with some tags:

- **Field layout mode:** on a field whose tag positions match the field layout selected in PhotonVision's settings. No setup needed, but practice fields are often not accurate enough.
- **Self-survey mode (recommended):** print 6–12 AprilTags (36h11, the same family your pipeline detects). Mount them flat and rigid, on foam board or plywood, around a roughly 3×3 m area. Vary their heights and angles so each camera sees at least two tags from most robot headings. You do not need to measure where the tags are; PhotonVision estimates their positions. You do need:
  - **The exact tag size.** Measure the black square of a printed tag with calipers. A 1% error in tag size gives a 1% error in camera position.
  - **A height reference.** Lay at least one tag flat on the floor, face up, where a camera can see it. Or measure the height of the center of one or two wall tags above the floor with a tape measure. Enter these IDs when solving.
  - **Unique tag IDs.** Never use the same ID twice.

## 3. Capture

1. Open **Robot Calibration** in the PhotonVision sidebar. Check that the robot shows as connected and that every camera shows as calibrated.
2. Enter the tag size and click **Start**.
3. Drive the robot in a stop-and-go pattern, pausing about a second at each stop. PhotonVision records a stop each time the robot settles after moving at least 15 cm or 15°.
   - Rotate in place to many headings (every 30–45° all the way around).
   - Drive short (30–50 cm) straight legs forward/back **and** sideways between rotations.
   - Repeat at 3–4 spots in the area.
   - Aim for 40+ stops, with all 12 heading buckets covered.
4. Click **Pause** when done. You can export the session as a dataset to re-solve later or to attach to a bug report.

:::{tip}
Only spinning in place can't determine camera yaw, and only driving straight can't determine camera x and y. Do both!
:::

## 4. Solve and save

Choose the mode (field layout or self-survey), enter floor tag IDs and/or measured tag heights, and click **Solve**. Check the results:

- **RMS reprojection error** should be around 1 px or less. Higher values usually mean a wrong tag size, a bad intrinsic calibration, or a duplicated tag ID.
- **±1σ** should be a few millimeters and a few tenths of a degree. Any camera with a warning needs more data: the warning says which kind of motion is missing.
- **Odometry scale** near 1.0 means odometry and tags agree on distances. A few percent off usually means the wheel radius in your code is wrong (or the tag size is).
- Compare against your CAD values, which are shown next to the result. Large differences are worth a second look at the mount.

Click **Save to cameras** to store the transforms on the coprocessor. You can also view or type a transform by hand in each camera's settings.

## 5. Use it in robot code

PhotonVision publishes the stored transform to NetworkTables, so robot code can read it instead of hard-coding it:

```{eval-rst}
.. tab-set-code::

   .. code-block:: java

      Transform3d robotToCam = camera.getRobotToCamera().orElse(kRobotToCamFromCad);
      var estimator = new PhotonPoseEstimator(fieldLayout, robotToCam);

   .. code-block:: c++

      wpi::math::Transform3d robotToCam =
          camera.GetRobotToCamera().value_or(kRobotToCamFromCad);

   .. code-block:: python

      robotToCam = camera.getRobotToCamera() or kRobotToCamFromCad
```

The transform is only available once the coprocessor has connected, so read it after startup (or periodically) and update the estimator with `setRobotToCameraTransform` if it changes. Many teams prefer to copy the calibrated values into their constants instead, which also works fine.

## Tips

- Recalibrate after anything touches a camera mount.
- Cameras on moving mechanisms (turrets, elevators) are not supported.
- The robot origin is the center of your drivetrain kinematics, at floor level. If your code uses a different origin, apply the offset yourself.
