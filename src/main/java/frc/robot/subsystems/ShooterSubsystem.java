package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.Supplier;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;


public class ShooterSubsystem extends SubsystemBase
{

  private final SparkFlex flywheelMotor  = new SparkFlex(Constants.IDConstants.Shooter_Left_ID, MotorType.kBrushless);
  private final SparkFlex flywheelMotor2 = new SparkFlex(Constants.IDConstants.Shooter_Right_ID, MotorType.kBrushless);

  private final SmartMotorControllerConfig motorConfig2 = new SmartMotorControllerConfig(this)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
      .withIdleMode(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(80))
      .withMotorInverted(true)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withMotorInverted(true)
      .withFeedforward(new SimpleMotorFeedforward(0.12,0.105,0.15))
      .withClosedLoopController(0.015,0,0.04)
      .withTelemetry("FlywheelMotor2",TelemetryVerbosity.HIGH);

  private final SmartMotorController motor2 = new SparkWrapper(flywheelMotor2, DCMotor.getNEO(1), motorConfig2);
  private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
      .withClosedLoopController(0.015, 0, 0.1)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
      .withIdleMode(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(80))
      .withMotorInverted(false)
      .withFeedforward(new SimpleMotorFeedforward(0.18, 0.105, 0.15))
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withTelemetry("FlyWheelMotor",TelemetryVerbosity.HIGH)
      .withLooselyCoupledFollowers(motor2);
  
  private final SmartMotorController motor  = new SparkWrapper(flywheelMotor, DCMotor.getNEO(1), motorConfig);


  public ShooterSubsystem()
  {
  }


  public AngularVelocity getFlywheel1Velocity()
  {
    return motor.getMechanismVelocity();
  }

  public AngularVelocity getFlywheel2Velocity()
  {
    return motor2.getMechanismVelocity();
  }

  public Command setVelocity(AngularVelocity speed)
  {
    return run(() -> motor.setVelocity(speed));
  }

  public Command setDutyCycle(double dutyCycle)
  {
    return run(() -> motor.setDutyCycle(dutyCycle));
  }

  public Command setVelocity(Supplier<AngularVelocity> speed)
  {
    return run(() -> motor.setVelocity(speed.get()));
  }

  public Command setDutyCycle(Supplier<Double> dutyCycle)
  {
    return run(() -> motor.setDutyCycle(dutyCycle.get()));
  }


  public void setTargetRPM(double rpm)
  {

    motor.setVelocity(RPM.of(rpm));
    // motor2.setVelocity(RPM.of(rpm));
  }

  public void stop()
  {

    motor.setDutyCycle(0);
    motor2.setDutyCycle(0);
  }

  public void periodic()
  {
    motor.updateTelemetry();
    motor2.updateTelemetry();
  }

  public void simulationPeriodic()
  {
    motor.simIterate();
    motor2.simIterate();
  }
}
