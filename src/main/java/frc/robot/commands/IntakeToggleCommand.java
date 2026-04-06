package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSpinSubsystem;

public class IntakeToggleCommand extends Command {

    private final IntakeSubsystem intake;
    //private final IntakeSpinSubsystem spin;

    // true = deployed, false = stowed
    private boolean deployed;

    public IntakeToggleCommand(IntakeSubsystem intake/* , IntakeSpinSubsystem spin*/) {
        this.intake = intake;
       // this.spin = spin;
        addRequirements(intake/* , spin*/);

        // start in stowed
        deployed = false;
    }

    @Override
    public void initialize() {
        // flip state
        deployed = !deployed;

        if (deployed) {
            // schedule the arm command ONCE
            intake.setAngle(Degrees.of(130)).schedule();
           // spin.runIntake(1);
        } else {
            intake.setAngle(Degrees.of(0)).schedule();
           // spin.stopIntake();
        }
    }

    @Override
    public boolean isFinished() {
        // finish immediately — we do NOT want to continuously hold
        return true;
    }
}