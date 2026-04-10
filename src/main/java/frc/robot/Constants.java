// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.Dashboard;
import java.util.function.Supplier;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{


  public static final boolean disableHAL        = false;
  public static final double  ROBOT_MASS        = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter  CHASSIS           = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)),
                                                             ROBOT_MASS);
  public static final double  LOOP_TIME         = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double  MAX_SPEED         = Units.feetToMeters(14.5);
  public static final double  MAX_ANGULAR_SPEED = Units.degreesToRadians(185);

  public static final class FieldConstants
  {

    /**
     * Blue alliance hub/speaker position (x, y) in inches.
     */
    public static final Translation2d BLUE_HUB_POSITION = new Translation2d(Inches.of(182.11).in(Meters),
                                                                            Inches.of(158.84).in(Meters));
    /**
     * Red alliance hub/speaker position (x, y) in inches.
     */
    public static final Translation2d RED_HUB_POSITION  = new Translation2d(Inches.of(469.11).in(Meters),
                                                                            Inches.of(158.84).in(Meters));
    public static final double yHalfLine = Inches.of(317 / 2).in(Meters);
    public static Pose2d                  blueHubPose = new Pose2d(BLUE_HUB_POSITION, Rotation2d.fromDegrees(0));
    public static Pose2d                  redHubPose  = new Pose2d(RED_HUB_POSITION, Rotation2d.fromDegrees(0));
    public static Supplier<Translation2d> hubPosition = () -> {
      return Dashboard.getAlliance() == Alliance.Blue ? BLUE_HUB_POSITION : RED_HUB_POSITION;
    };
  }
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled

  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND         = 0.1;
    public static final double LEFT_Y_DEADBAND  = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

  public static class IDConstants
  {

    public static final int Shooter_Left_ID   = 9;
    public static final int Shooter_Right_ID  = 10;
    public static final int IntakearmMotor_ID = 11;
    public static final int IntakeMotor_ID    = 12;
    public static final int IndexerMotor_ID   = 13;
    public static final int KickerMotor_ID    = 14;
    public static final int KickerMotor_ID2   = 15;
    public static final int ClimberMotor_ID   = 16;
  }

  public static class ShooterConstants
  {

    public static final double FARShooterGoalRPM   = 2600;
    public static final double NEARShooterGoalRPM  = 3000;
    public static final double ShooterToleranceRPM = 100;
    public static final double Shooter_Speed       = -0.7;
    public static final double kShooterRampRate    = 0.1;
  }

  public static class IntakeConstants
  {

    public static final int IntakeOutAngle = 10;
    public static final int IntakeinAngle  = 90;
    public static final int IntakeSpeed    = 1;

  }

  public static class IndexConstants
  {

    public static final double IndexSpeed = -1;

  }

  public static class KickerConstants
  {

    public static final double KickSpeed          = 1;
    public static final double KickerGoalRPM      = 1000;
    public static final double KickerToleranceRPM = 1500;
  }

  public static class DriveConstants
  {

    public static final double maxTurnSpeed = 480; // Degrees
    /**
     * Maximum translational speed of the robot in meters per second.
     */
    public static final double maxSpeed     = 6; // Meters per second

    /**
     * PID constants for translational (x, y) movement control. Pathplanner
     */
    public static final PIDConstants pptranslationPidConstants = new PIDConstants(5, 0, 0);
    /**
     * PID constants for rotational (theta) movement control. Pathplanner
     */
    public static final PIDConstants pprotationPidConstants    = new PIDConstants(8, 0, 0);

    /**
     * PID constants for translational (x, y) movement control.
     */
    public static final PIDConstants translationPidConstants  = new PIDConstants(4.5, 0, 1);
    /**
     * PID constants for rotational (theta) movement control.
     */
    public static final PIDConstants rotationPidConstants     = new PIDConstants(6, 0, 0);
    /**
     * Motion profile constraints for translation (meters/sec, meters/sec^2)
     */
    public static final double       TRANSLATION_MAX_VELOCITY = maxSpeed; // m/s
    public static final double       TRANSLATION_MAX_ACCEL    = 3.0; // m/s^2 (tunable)

    public static final ProfiledPIDController translationProfiledController =
        new ProfiledPIDController(
            translationPidConstants.kP, translationPidConstants.kI, translationPidConstants.kD,
            new TrapezoidProfile.Constraints(TRANSLATION_MAX_VELOCITY, TRANSLATION_MAX_ACCEL)
        );
    /**
     * Motion profile constraints for rotation (radians/sec, radians/sec^2)
     */
    public static final double ROTATION_MAX_VELOCITY = Math.toRadians(maxTurnSpeed); // rad/s
    public static final double ROTATION_MAX_ACCEL    = Math.toRadians(maxTurnSpeed * 2.0); // rad/s^2 (tunable)
    public static final ProfiledPIDController rotationProfiledController =
        new ProfiledPIDController(
            rotationPidConstants.kP, rotationPidConstants.kI, rotationPidConstants.kD,
            new TrapezoidProfile.Constraints(ROTATION_MAX_VELOCITY, ROTATION_MAX_ACCEL)
        );

  }
}