// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

/** Command to auto-align to an AprilTag using the Limelight via the DriveSubsystem */
public class LimelightAlignCommand extends Command {

    private final Command alignCommand;

    /** Creates a new DriveToLimelightTargetCommand. */
    public LimelightAlignCommand(SwerveSubsystem drivebase) {
        // Store the command returned by your subsystem function
        this.alignCommand = drivebase.driveToLimelightTarget();

        // Declare subsystem requirements
        addRequirements(drivebase);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // Schedule the command from the subsystem
        alignCommand.schedule();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Nothing needed; the subsystem command handles execution
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // Stop the subsystem command if this wrapper is interrupted
        alignCommand.cancel();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // Finish when the subsystem command finishes
        return alignCommand.isFinished();
    }
}
