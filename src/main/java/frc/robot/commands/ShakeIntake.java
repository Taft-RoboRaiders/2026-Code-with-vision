package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.IntakeSubsystem;

public class ShakeIntake
{

  public static Command shake(IntakeSubsystem intake)
  {

    return Commands.sequence(
        intake.setAngle(Degrees.of(320)).withTimeout(0.15),
        intake.setAngle(Degrees.of(350)).withTimeout(0.15)
                            ).repeatedly().withTimeout(2); // shake for 2 seconds
  }
}