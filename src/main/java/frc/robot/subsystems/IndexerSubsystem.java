package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexerSubsystem extends SubsystemBase
{

  private final SparkMax indexerMotor =
      new SparkMax(Constants.IDConstants.IndexerMotor_ID, MotorType.kBrushless);


  public IndexerSubsystem()
  {

  }

  /**
   * Set indexer speed (only command leader)
   */
  public void runIndexer(double speed)
  {
    indexerMotor.set(speed);
  }

  /**
   * Stop indexer
   */
  public void stopIndexer()
  {
    indexerMotor.set(0);
  }

  /**
   * Command: run indexer at a fixed speed
   */
  public Command runIndexerCommand(double speed)
  {
    return run(() -> runIndexer(speed));
  }

  /**
   * Command: stop indexer
   */
  public Command stopIndexerCommand()
  {
    return runOnce(this::stopIndexer);
  }

  @Override
  public void periodic()
  {
    SmartDashboard.putNumber("IndexerLeaderRPM", indexerMotor.getEncoder().getVelocity());

  }
}