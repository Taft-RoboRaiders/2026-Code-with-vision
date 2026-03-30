package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public class IntakeSpinSubsystem extends SubsystemBase
{

  private final SparkMax m_IntakeSpin = new SparkMax(Constants.IDConstants.IntakeMotor_ID, MotorType.kBrushless);

  private final SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.OPEN_LOOP)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
      .withTelemetry("IntakeSpinMotor", TelemetryVerbosity.HIGH)
      .withMotorInverted(true)
      .withIdleMode(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(60));


  // Run intake roller
  public void runIntake(double speed)
  {
    m_IntakeSpin.set(speed);
  }

  // Stop intake roller
  public void stopIntake()
  {
    m_IntakeSpin.set(0);
  }

  // Command to run intake
  public Command runIntakeCommand(double speed)
  {
    return run(() -> runIntake(speed));
  }

  // Command to stop intake
  public Command stopIntakeCommand()
  {
    return run(this::stopIntake);
  }

   @Override
  public void periodic()
  {
    SmartDashboard.putNumber("IntakeSpeedRPM", m_IntakeSpin.getEncoder().getVelocity());

  }

}
