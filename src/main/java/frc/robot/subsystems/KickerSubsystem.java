package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
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

public class KickerSubsystem extends SubsystemBase
{

  private final SparkMax KickerMotor =
      new SparkMax(Constants.IDConstants.KickerMotor_ID, MotorType.kBrushless);

  private final SparkMax KickerMotor2 =
      new SparkMax(Constants.IDConstants.KickerMotor_ID2, MotorType.kBrushless);

  private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
      .withClosedLoopController(0.015, 0, 0.1)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(3)))
      .withIdleMode(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(60))
      .withMotorInverted(true)
      .withFeedforward(new SimpleMotorFeedforward(0.10, 0.123, 0.015))
      .withControlMode(ControlMode.CLOSED_LOOP);

  private final SmartMotorController motor2 = new SparkWrapper(KickerMotor2, DCMotor.getNEO(1), motorConfig.clone()
                                                                                                           .withMotorInverted(
                                                                                                               false)
                                                                                                           .withFeedforward(
                                                                                                               new SimpleMotorFeedforward(
                                                                                                                   0.125,
                                                                                                                   0.127,
                                                                                                                   0.015))
                                                                                                           .withClosedLoopController(
                                                                                                               0.008,
                                                                                                               0,
                                                                                                               0.02)

                                                                                                           .withTelemetry(
                                                                                                               "KickerMotor2",
                                                                                                               TelemetryVerbosity.HIGH)
  );

  private final SmartMotorController kickmotor = new SparkWrapper(KickerMotor, DCMotor.getNEO(2),
                                                                  motorConfig.withTelemetry("KickerMotor",
                                                                                            TelemetryVerbosity.HIGH));

  public KickerSubsystem()
  {
    KickerMotor.pauseFollowerMode();
    KickerMotor2.pauseFollowerMode();
  }

  public AngularVelocity getFlywheel1Velocity()
  {
    return kickmotor.getMechanismVelocity();
  }

  public AngularVelocity getFlywheel2Velocity()
  {
    return motor2.getMechanismVelocity();
  }

  public Command setVelocity(AngularVelocity speed)
  {
    return run(() -> kickmotor.setVelocity(speed));
  }

  public Command setDutyCycle(double dutyCycle)
  {
    return run(() -> kickmotor.setDutyCycle(dutyCycle));
  }

  public Command setVelocity(Supplier<AngularVelocity> speed)
  {
    return run(() -> kickmotor.setVelocity(speed.get()));
  }

  public Command setDutyCycle(Supplier<Double> dutyCycle)
  {
    return run(() -> kickmotor.setDutyCycle(dutyCycle.get()));
  }


  public void setTargetRPM(double rpm)
  {

    kickmotor.setVelocity(RPM.of(rpm));
    motor2.setVelocity(RPM.of(rpm));
  }

  public void stop()
  {

    kickmotor.setDutyCycle(0);
    motor2.setDutyCycle(0);
  }

  public void periodic()
  {
    kickmotor.updateTelemetry();
    motor2.updateTelemetry();
  }

  public void simulationPeriodic()
  {
    kickmotor.simIterate();
    motor2.simIterate();
  }
}

