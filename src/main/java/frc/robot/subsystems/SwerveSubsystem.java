// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meter;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import java.io.File;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import limelight.Limelight;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.networktables.LimelightResults;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;


public class SwerveSubsystem extends SubsystemBase
{

  private static final double                POSE_RENDER_Z = 0.9;
  private static final Set<Integer> ALLOWED_TAG_IDS = Set.of(9, 10, 25, 26);
  private final        SwerveDrive           swerveDrive;
  private final        CommandXboxController driverController;
  private final        DoubleSupplier        getTranslationX;
  private final        DoubleSupplier        getTranslationY;
  private final        DoubleSupplier        getHeadingX;
  private final        DoubleSupplier        getHeadingY;
  private final Pose2d blueAllianceStartingPose = new Pose2d(new Translation2d(Meter.of(1),
                                                                               Meter.of(4)),
                                                             Rotation2d.fromDegrees(0));
  private final Pose2d redAllianceStartingPose  = new Pose2d(new Translation2d(Meter.of(16),
                                                                               Meter.of(4)),
                                                             Rotation2d.fromDegrees(180));
  /**
   * Swerve drive object.
   */
  public SwerveInputStream baseStream;
  SwerveDrivePoseEstimator SwerveDrivePoseEstimator;
  SwerveDrivePoseEstimator buildDriveToCurveStream;
  Limelight              limelight;
  LimelightPoseEstimator limelightPoseEstimator;
  private volatile boolean driveToPoseActive = false;
  private int     outofAreaReading = 0;
  private boolean initialReading   = false;
  private double  lastLLTimestamp  = 0;

  /**
   * Initialize {@link SwerveDrive} with the directory provided.
   *
   * @param directory Directory of swerve drive config files.
   */


  public SwerveSubsystem(CommandXboxController driverController)
  {
    this.driverController = driverController;

    getTranslationX = () -> -MathUtil.applyDeadband(driverController.getLeftY(), 0.1);
    getTranslationY = () -> -MathUtil.applyDeadband(driverController.getLeftX(), 0.1);

    getHeadingX = () -> -driverController.getRightX();
    getHeadingY = () -> -driverController.getRightY();

    boolean blueAlliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
                           == DriverStation.Alliance.Blue;

    Pose2d startingPose = blueAlliance ? blueAllianceStartingPose : redAllianceStartingPose;

    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

    File directory = new File(Filesystem.getDeployDirectory(), "swerve");

    try
    {
      swerveDrive = new SwerveParser(directory)
          .createSwerveDrive(Constants.MAX_SPEED, startingPose);
    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }

    baseStream = SwerveInputStream.of(swerveDrive, getTranslationX, getTranslationY)
                                  .allianceRelativeControl(true)
                                  .deadband(0.12);

    setupLimelight();
  }

  public void setupLimelight()
  {
    swerveDrive.stopOdometryThread();
    limelight = new Limelight("limelight");
    limelight
        .getSettings()
        .withPipelineIndex(0)
        .withCameraOffset(
            new Pose3d(
                Units.inchesToMeters(13),
                Units.inchesToMeters(13.5),
                Units.inchesToMeters(22.5),
                new Rotation3d(0, 30, Units.degreesToRadians(180))))
        .withAprilTagIdFilter(List.of(25, 26, 19, 10, 18, 27, 21, 24, 5, 8, 2, 11))
        .save();
    limelightPoseEstimator =
        limelight.createPoseEstimator(EstimationMode.MEGATAG2);
  }

  public SwerveInputStream buildDefaultStream()
  {
    return baseStream.copy()
                     .withControllerHeadingAxis(getHeadingX, getHeadingY)
                     .headingWhile(true);
  }

  public SwerveInputStream buildRelativeTurningStream()
  {
    return baseStream
        .copy()
        .withControllerRotationAxis(() -> -driverController.getRightX());
  }

  public void setDriveToPoseActive(boolean active) {this.driveToPoseActive = active;}

  public Pose2d getClosestPointOnCurve(double controllerOffset)
  {
    // p = robot position, h = hub position, d = desired distance (midRange)
    Translation2d p = getPose().getTranslation();
    Translation2d h = Constants.FieldConstants.hubPosition.get();

    SmartDashboard.putNumber("ControllerOffset", controllerOffset);

    // vector from hub to robot: v = p - h
    Translation2d v = p.minus(h);

    // distance ||v||
    double dist = Math.hypot(v.getX(), v.getY());

    // Guard against division by zero (robot exactly at hub)
    if (dist < 1e-6)
    {
      // undefined direction; return a pose at the current position pointing at the hub
      return posePointingAtAllianceHub(p);
    }

    // ShooterSubsystemConstants.midRange is in meters (use directly)
    double desiredDistanceMeters = 2.8;

    // factor = d / ||v||
    double factor = desiredDistanceMeters / dist;

    // target = h + factor * v  -> matches: d/||p-h|| * (p-h) + (h_x,h_y)
    Translation2d continuousTarget = v.times(factor).plus(h);

    // Publish continuous target distance to hub (should be equal to desiredDistanceMeters)
    double continuousTargetDist = Math.hypot(continuousTarget.getX() - h.getX(), continuousTarget.getY() - h.getY());
    SmartDashboard.putNumber("Drive/ContinuousTargetDistanceFromHubMeters", continuousTargetDist);

    // Use the controller offset to slide left/right along the hub arc.
    // Map controllerOffset (-1..1) to an angular offset around the hub. We choose
    // a max angular excursion of +/- 90 degrees (pi/2) so the driver can move left/right
    // along the arc without flipping to the far side.
    double baseAngle   = Math.atan2(v.getY(), v.getX()); // angle from hub->robot
    double maxAngle    = Math.PI / 2.0; // +/- 90 degrees
    double angleOffset = MathUtil.clamp(controllerOffset, -1.0, 1.0) * maxAngle;
    double chosenTheta = baseAngle + angleOffset;

    // Compensate the aiming angle slightly opposite the robot's horizontal travel to
    // counteract momentum. Compute lateral sign relative to the hub->robot vector and
    // scale compensation by current speed fraction (0..1) with a small max angle.
    try
    {
      var    vel         = swerveDrive.getFieldVelocity();
      double vx          = vel.vxMetersPerSecond;
      double vy          = vel.vyMetersPerSecond;
      double travelSpeed = Math.hypot(vx, vy);
      double travelAngle = Math.atan2(vy, vx);

      // relative angle from hub->robot to travel vector
      double rel = travelAngle - baseAngle;

      // scale: fraction of max chassis speed (guard positive)
      double speedFraction = 0.0;
      double maxSpeed      = swerveDrive.getMaximumChassisVelocity();
      if (maxSpeed > 1e-6) {speedFraction = MathUtil.clamp(travelSpeed / maxSpeed, 0.0, 1.0);}

      // make compensation stronger when driver offsets further from center: scale by |controllerOffset|
      double offsetFactor = Math.abs(MathUtil.clamp(controllerOffset, -1.0, 1.0));
      double offsetGain   = 6.0; // stronger multiplier to make offset influence very noticeable

      // maximum compensation angle (radians) at full speed and full offset
      double maxCompensation = Math.toRadians(40.0); // substantially larger ( ~40 degrees )

      // scale compensation proportionally with lateral motion magnitude so straighter
      // forward/back motion doesn't add huge compensation. Use lateral magnitude (0..1).
      double lateralMagnitude = Math.abs(Math.sin(rel));

      // Compose final compensation: sign * speed fraction * offset factor * lateral magnitude * gain * max
      double compensation =
          -Math.signum(Math.sin(rel)) * speedFraction * offsetFactor * lateralMagnitude * offsetGain * maxCompensation;
      chosenTheta += compensation;

      SmartDashboard.putNumber("Drive/TravelSpeed", travelSpeed);
      SmartDashboard.putNumber("Drive/TravelAngleRad", travelAngle);
      SmartDashboard.putNumber("Drive/AngleCompensationRad", compensation);
      SmartDashboard.putNumber("Drive/OffsetFactor", offsetFactor);
      SmartDashboard.putNumber("Drive/OffsetGain", offsetGain);
    } catch (Exception ex)
    {
      // if anything goes wrong (e.g., swerveDrive not ready), skip compensation
    }

    Translation2d snappedTarget = new Translation2d(
        h.getX() + desiredDistanceMeters * Math.cos(chosenTheta),
        h.getY() + desiredDistanceMeters * Math.sin(chosenTheta)
    );

    double snappedTargetDist = Math.hypot(snappedTarget.getX() - h.getX(), snappedTarget.getY() - h.getY());
    SmartDashboard.putNumber("Drive/SnappedTargetDistanceFromHubMeters", snappedTargetDist);
    SmartDashboard.putNumber("Drive/DesiredDistanceMeters", desiredDistanceMeters);
    SmartDashboard.putNumber("Drive/TargetDistanceErrorMeters", snappedTargetDist - desiredDistanceMeters);
    SmartDashboard.putNumber("Drive/HubArcChosenAngleRad", chosenTheta);
    SmartDashboard.putNumber("Drive/ControllerOffset", controllerOffset);

    return posePointingAtAllianceHub(snappedTarget);
  }

  public SwerveInputStream buildDriveToCurveStream()
  {
    return baseStream
        .copy()
        .driveToPose(
            // Use a deadbanded left-stick X so the driver can move left/right along the curve
            () -> {
              double raw    = driverController.getLeftX();
              double offset = MathUtil.applyDeadband(raw, 0.1) / 2;
              return getClosestPointOnCurve(offset);
            },
            Constants.DriveConstants.translationProfiledController,
            Constants.DriveConstants.rotationProfiledController
                    )
        .driveToPoseEnabled(true);
  }

  public Pose2d posePointingAtAllianceHub(Translation2d position)
  {
    Translation2d hubPosition = frc.robot.Constants.FieldConstants.hubPosition.get();

    // Publish hub location for debugging as Pose3d: [x, y, z, rollDeg, pitchDeg, yawDeg]
    SmartDashboard.putNumberArray("Drive/HubPose",
                                  new double[]{hubPosition.getX(), hubPosition.getY(), POSE_RENDER_Z, 0.0, 0.0, 0.0});

    // Angle from the given position to the hub (robot should point at the hub)
    double angle = Math.atan2(hubPosition.getY() - position.getY(), hubPosition.getX() - position.getX());

    // Create a pose at the provided position with rotation toward the hub.
    // IMPORTANT: do NOT add the hub translation again; that would produce position + hub which is incorrect.
    Pose2d targetPose = new Pose2d(position, new Rotation2d(angle).rotateBy(Rotation2d.kZero));

    // Prepare the arc pose3d for publishing (default)
    Pose3d arcPose3d = new Pose3d(new Translation3d(targetPose.getX(), targetPose.getY(), POSE_RENDER_Z),
                                  new Rotation3d(0.0, 0.0, targetPose.getRotation().getRadians()));
    SmartDashboard.putNumberArray("Drive/TargetPoseArc",
                                  new double[]{arcPose3d.getX(), arcPose3d.getY(), arcPose3d.getZ(),
                                               Math.toDegrees(arcPose3d.getRotation().getX()),
                                               Math.toDegrees(arcPose3d.getRotation().getY()),
                                               Math.toDegrees(arcPose3d.getRotation().getZ())});

    // Try to find a projectile close to the snapped target; if present, publish the ball Pose3d as the target
    Pose3d chosenPose3d = arcPose3d;

    // Publish the chosen target pose (ball if present, otherwise arc)
    SmartDashboard.putNumberArray("Drive/TargetPose",
                                  new double[]{chosenPose3d.getX(), chosenPose3d.getY(), chosenPose3d.getZ(),
                                               Math.toDegrees(chosenPose3d.getRotation().getX()),
                                               Math.toDegrees(chosenPose3d.getRotation().getY()),
                                               Math.toDegrees(chosenPose3d.getRotation().getZ())});

    return targetPose;
  }

  public Command driveWithChassisSpeedsSupplier(Supplier<ChassisSpeeds> speedsSupplier)
  {
    return run(() -> {

      ChassisSpeeds speeds = speedsSupplier.get();

      Translation2d translation =
          new Translation2d(
              speeds.vxMetersPerSecond,
              speeds.vyMetersPerSecond
          );

      double omega = speeds.omegaRadiansPerSecond;

      // Drive robot-relative
      swerveDrive.drive(translation, omega, true, false);

    });
  }

  @Override
  public void periodic()
  {
    swerveDrive.updateOdometry();
    limelight
        .getSettings()
        .withRobotOrientation(
            new Orientation3d(
                new Rotation3d(swerveDrive.getOdometryHeading().rotateBy(Rotation2d.kZero)),
                new AngularVelocity3d(DegreesPerSecond.of(0), DegreesPerSecond.of(0), DegreesPerSecond.of(0))))
        .save();

    Optional<PoseEstimate> poseEstimates =
        limelightPoseEstimator.getPoseEstimate();
    Optional<LimelightResults> results = limelight.getLatestResults();
    if (results.isPresent() && poseEstimates.isPresent())
    {
      LimelightResults result       = results.get();
      PoseEstimate     poseEstimate = poseEstimates.get();
      SmartDashboard.putNumber("limelight/Avg Tag Ambiguity",
                               poseEstimate.getAvgTagAmbiguity());
      SmartDashboard.putNumber("limelight/Min Tag Ambiguity",
                               poseEstimate.getMinTagAmbiguity());
      SmartDashboard.putNumber("limelight/Max Tag Ambiguity",
                               poseEstimate.getMaxTagAmbiguity());
      SmartDashboard.putNumber("limelight/Avg Distance", poseEstimate.avgTagDist);
      SmartDashboard.putNumber("limelight/Avg Tag Area", poseEstimate.avgTagArea);
      SmartDashboard.putNumber("limelight/Odom/Pose/x", swerveDrive.getPose().getX());
      SmartDashboard.putNumber("limelight/Odom/Pose/y", swerveDrive.getPose().getY());
      SmartDashboard.putNumber(
          "limlelight/Odom/Pose/degrees", swerveDrive.getPose().getRotation().getDegrees());
      SmartDashboard.putNumber("limelight/Pose/x", poseEstimate.pose.getX());
      SmartDashboard.putNumber("limelight/Pose/y", poseEstimate.pose.getY());
      SmartDashboard.putNumber(
          "limelight/Pose/degrees",
          poseEstimate.pose.toPose2d().getRotation().getDegrees());
      if (result.valid)
      {
        Pose2d estimatorPose = poseEstimate.pose.toPose2d();
        Pose2d usefulPose    = result.getBotPose2d(Alliance.Blue);
        double distanceToPose =
            usefulPose.getTranslation().getDistance(swerveDrive.getPose().getTranslation());
        if (distanceToPose < 0.5
            || (outofAreaReading > 10)
            || (outofAreaReading > 10 && !initialReading))
        {
          if (!initialReading)
          {
            initialReading = true;
          }
          outofAreaReading = 0;
          if (lastLLTimestamp != result.timestamp_RIOFPGA_capture)
          {
            var stdDevScale = Math.pow(result.botpose_avgdist, 2.0) / result.botpose_tagcount;
            // stdDevScale = distance^2/tagsInView
            swerveDrive.setVisionMeasurementStdDevs(VecBuilder.fill(0.05 * stdDevScale,
                                                                    0.05 * stdDevScale,
                                                                    0.022 * stdDevScale));
            swerveDrive.addVisionMeasurement(usefulPose,
                                             result.timestamp_RIOFPGA_capture);
            lastLLTimestamp = result.timestamp_RIOFPGA_capture;
          }
        } else
        {
          outofAreaReading += 1;
        }
        swerveDrive.addVisionMeasurement(estimatorPose,
                                         poseEstimate.timestampSeconds);
      }
    }
  }

  @Override
  public void simulationPeriodic()
  {

  }


  public void setupPathPlanner()

  {
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config;
    try
    {
      config = RobotConfig.fromGUISettings();

      final boolean enableFeedforward = true;
      // Configure AutoBuilder last
      AutoBuilder.configure(
          swerveDrive::getPose,
          // Robot pose supplier
          swerveDrive::resetOdometry,
          // Method to reset odometry (will be called if your auto has a starting pose)
          swerveDrive::getRobotVelocity,
          // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speedsRobotRelative, moduleFeedForwards) -> {
            if (enableFeedforward)
            {
              swerveDrive.drive(
                  speedsRobotRelative,
                  swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                  moduleFeedForwards.linearForces()
                               );
            } else
            {
              swerveDrive.setChassisSpeeds(speedsRobotRelative);
            }
          },
          // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
          new PPHolonomicDriveController(
              // PPHolonomicController is the built in path following controller for holonomic drive trains
              new PIDConstants(5.0, 0.0, 0.0),
              // Translation PID constants
              new PIDConstants(5.0, 0.0, 0.0)
              // Rotation PID constants
          ),
          config,
          // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent())
            {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this
          // Reference to this subsystem to set requirements
                           );

    } catch (Exception e)
    {
      // Handle exception as needed
      e.printStackTrace();
    }

    //Preload PathPlanner Path finding
    // IF USING CUSTOM PATHFINDER ADD BEFORE THIS LINE
    PathfindingCommand.warmupCommand().schedule();
  }

  public Command getAutonomousCommand(String pathName)
  {
    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return new PathPlannerAuto(pathName);
  }
  /**
   * Get the path follower with events.
   *
   * @param pathName PathPlanner path name.
   * @return {@link AutoBuilder#followPath(PathPlannerPath)} path command.
   */

  /**
   * Returns a Command that centers the modules of the SwerveDrive subsystem.
   *
   * @return a Command that centers the modules of the SwerveDrive subsystem
   */
  public Command centerModulesCommand()
  {
    return run(() -> Arrays.asList(swerveDrive.getModules())
                           .forEach(it -> it.setAngle(0.0)));
  }

  /**
   * Returns a Command that tells the robot to drive forward until the command ends.
   *
   * @return a Command that tells the robot to drive forward until the command ends
   */
  public Command driveForward()
  {
    return run(() -> {
      swerveDrive.drive(new Translation2d(1, 0), 0, false, false);
    }).finallyDo(() -> swerveDrive.drive(new Translation2d(0, 0), 0, false, false));
  }

  public Command stopCommand()
  {
    return run(() -> {
      swerveDrive.drive(new Translation2d(0, 0), 0, false, false);
    }).finallyDo(() -> swerveDrive.drive(new Translation2d(0, 0), 0, false, false));
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */

  public void driveFieldOriented(ChassisSpeeds velocity)
  {
    swerveDrive.driveFieldOriented(velocity);
  }


  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity)
  {
    return run(() -> {
      swerveDrive.driveFieldOriented(velocity.get());
    });
  }


  /**
   * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this
   * method.  However, if either gyro angle or module position is reset, this must be called in order for odometry to
   * keep working.
   *
   * @param initialHolonomicPose The pose to set the odometry to
   */
  public void resetOdometry(Pose2d initialHolonomicPose)
  {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  /**
   * Gets the current pose (position and rotation) of the robot, as reported by odometry.
   *
   * @return The robot's pose
   */
  public Pose2d getPose()
  {
    return swerveDrive.getPose();
  }

  /**
   * Set chassis speeds with closed-loop velocity control.
   *
   * @param chassisSpeeds Chassis Speeds to set.
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds)
  {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  /**
   * Post the trajectory to the field.
   *
   * @param trajectory The trajectory to post.
   */
  public void postTrajectory(Trajectory trajectory)
  {
    swerveDrive.postTrajectory(trajectory);
  }

  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
   */
  public void zeroGyro()
  {
    swerveDrive.zeroGyro();
  }

  /**
   * Gets the current yaw angle of the robot, as reported by the swerve pose estimator in the underlying drivebase.
   * Note, this is not the raw gyro reading, this may be corrected from calls to resetOdometry().
   *
   * @return The yaw angle
   */
  public Rotation2d getHeading()
  {
    return getPose().getRotation();
  }

  /**
   * Gets the swerve drive object.
   *
   * @return {@link SwerveDrive}
   */
  public SwerveDrive getSwerveDrive()
  {
    return swerveDrive;
  }


  /**
   * Get the distance to the hub in meters
   *
   * @return Distance to hub in meters.
   */
  public Double distanceToHub()
  {
    // p = robot position, h = hub position, d = desired distance (midRange)
    Translation2d p = getPose().getTranslation();
    Translation2d h = Constants.FieldConstants.hubPosition.get();

    // vector from hub to robot: v = p - h
    Translation2d v = p.minus(h);

    // distance ||v||
    double dist = Math.hypot(v.getX(), v.getY());

    SmartDashboard.putNumber("AutoShootRPM/distance/meters", dist);
    return dist;
  }
}