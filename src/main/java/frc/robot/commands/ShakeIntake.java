package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.IntakeSubsystem;

public class ShakeIntake {

    public static Command shake(IntakeSubsystem intake) {
        // sequence back and forth and repeat
        return Commands.sequence(
                intake.setAngle(Degrees.of(100)).withTimeout(0.5),
                intake.setAngle(Degrees.of(30)).withTimeout(0.5)
        ).repeatedly().withTimeout(2.5); // shake for 2.5 seconds
    }
}