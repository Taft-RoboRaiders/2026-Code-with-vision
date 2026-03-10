package frc.robot.Autonomous;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class Auto
{

  // Subsystems
  private final SwerveSubsystem  swerve;
  private final ShooterSubsystem shooter;
  private final IntakeSubsystem  intake;


  public Auto(SwerveSubsystem swerve, ShooterSubsystem shooter, IntakeSubsystem intake)
  {
    this.swerve = swerve;
    this.shooter = shooter;
    this.intake = intake;

  }
}
