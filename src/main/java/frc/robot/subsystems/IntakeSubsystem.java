package frc.robot.subsystems;


import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;


public class IntakeSubsystem extends SubsystemBase
{

  private final SparkMax IntakearmMotor = new SparkMax(Constants.IDConstants.IntakearmMotor_ID, MotorType.kBrushless);


  //Will tune this on Bot, Sim isnt being nice with YAMS arm tuning
  private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
      .withClosedLoopController(5, 0, 0.05)//, DegreesPerSecond.of(20), DegreesPerSecondPerSecond.of(20))  
      .withSoftLimit(Degrees.of(5), Degrees.of(140))
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(57.78)))
      .withIdleMode(MotorMode.BRAKE)
      .withTelemetry("IntakeMotor", TelemetryVerbosity.HIGH)
      .withStatorCurrentLimit(Amps.of(40))
      .withMotorInverted(false)
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withFeedforward(new ArmFeedforward(0.15, 0, 5, 2))
      .withControlMode(ControlMode.CLOSED_LOOP);


  private final SmartMotorController motor = new SparkWrapper(IntakearmMotor,
                                                              DCMotor.getNEO(1),
                                                              motorConfig);

  private final ArmConfig m_config = new ArmConfig(motor)
      .withLength(Meters.of(0.135))
      .withHardLimit(Degrees.of(0), Degrees.of(145))
      .withTelemetry("IntakeArm", TelemetryVerbosity.HIGH)
      .withMass(Pounds.of(10))
      .withStartingPosition(Degrees.of(0));

  private final Arm intakeArm = new Arm(m_config);

  public IntakeSubsystem()
  {

  }

  public void periodic()
  {
    intakeArm.updateTelemetry();
    SmartDashboard.putNumber("IntakeArmValue", IntakearmMotor.getEncoder().getPosition() * 360);

  }

  public void simulationPeriodic()
  {
    intakeArm.simIterate();
  }

  public Command armCmd(double dutycycle)
  {
    return intakeArm.set(dutycycle);
  }

  public Command sysId()
  {
    return intakeArm.sysId(Volts.of(3), Volts.of(3).per(Second), Second.of(30));
  }

  public Command setAngle(Angle angle)
  {
    return intakeArm.setAngle(angle);
  }

  public Command DeployIntake(Angle angle)
  {
    return intakeArm.setAngle(Degrees.of(0));  //DONT KNOW IF 25 IS RIGHT YET
  }

  public Command StowIntake(Angle angle)
  {
    return intakeArm.setAngle(Degrees.of(145));  //DONT KNOW IF 115 IS RIGHT YET
  }


}



