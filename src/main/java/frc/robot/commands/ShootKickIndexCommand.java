package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.List;
import java.util.Optional;

public class ShootKickIndexCommand extends Command
{

  private final ShooterSubsystem          shooter;
  private final KickerSubsystem           kicker;
  private final IndexerSubsystem          indexer;
  private final Optional<SwerveSubsystem> swerve;
  private final double goalRPM;   // <-- parameter stored here
  private final Debouncer shootDebounce1 = new Debouncer(0.3, DebounceType.kFalling);
  private final Debouncer shootDebounce2 = new Debouncer(0.3, DebounceType.kFalling);
  private final List<RecordedShot>         shots             = List.of(
      // TUNE HERE
      new RecordedShot(Meters.of(3.445), RPM.of(3600), Second.of(0.71)),
      new RecordedShot(Meters.of(4.571), RPM.of(4000), Second.of(1.04)),
      new RecordedShot(Meters.of(3.4527), RPM.of(3550), Second.of(0.78)),
      new RecordedShot(Meters.of(4.06), RPM.of(3800), Second.of(0.83)),
      new RecordedShot(Meters.of(4.638), RPM.of(4000), Second.of(1.08)),
      new RecordedShot(Meters.of(2.65135), RPM.of(3450), Second.of(0.64)),
      new RecordedShot(Meters.of(2.867095988657588), RPM.of(3300), Second.of(0.62))


          
        

                                                                      );
  private final InterpolatingDoubleTreeMap calculatedGoalRPM = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap calculatedTOF     = new InterpolatingDoubleTreeMap();
  public ShootKickIndexCommand(
      ShooterSubsystem shooter,
      KickerSubsystem kicker,
      IndexerSubsystem indexer,
      double goalRPM1            // <-- parameter passed in
                              )
  {
    this.shooter = shooter;
    this.kicker = kicker;
    this.indexer = indexer;
    this.swerve = Optional.empty();
    this.goalRPM = goalRPM1;   // <-- store parameter

    addRequirements(shooter, kicker, indexer);
  }

  public ShootKickIndexCommand(
      ShooterSubsystem shooter,
      KickerSubsystem kicker,
      IndexerSubsystem indexer,
      SwerveSubsystem swerve
                              )
  {
    this.shooter = shooter;
    this.kicker = kicker;
    this.indexer = indexer;
    this.swerve = Optional.of(swerve);
    goalRPM = 0;

    for (var shot : shots)
    {
      calculatedGoalRPM.put(shot.distance.in(Meters), shot.shooterSpeed.in(RPM));
      calculatedTOF.put(shot.distance.in(Meters), shot.tof.in(Second));
    }
    addRequirements(shooter, kicker, indexer);
  }

  @Override
  public void initialize()
  {
    // Spin up shooter to the passed RPM
  }

  @Override
  public void execute()
  {
    
    double goalRPM1 = goalRPM;
    if (swerve.isPresent())
    {
      goalRPM1 = swerve.get().distanceToHub();
    }

    shooter.setTargetRPM(goalRPM1);

    AngularVelocity shooterRPM  = shooter.getFlywheel1Velocity();
    AngularVelocity shooterRPM2 = shooter.getFlywheel2Velocity();

    boolean shooterReady = shootDebounce1.calculate(
        shooterRPM.isNear(
            RPM.of(goalRPM1),
            RPM.of(Constants.ShooterConstants.ShooterToleranceRPM)
                         )
                                                   ) && shootDebounce2.calculate(
        shooterRPM2.isNear(
            RPM.of(goalRPM1),
            RPM.of(Constants.ShooterConstants.ShooterToleranceRPM)
                          )
                                                                                );

    if (shooterReady)
    {
      kicker.runKicker(0.8);
      indexer.runIndexer(Constants.IndexConstants.IndexSpeed);
    } else
    {
      kicker.runKicker(0);
      indexer.stopIndexer();
    }
  }

  @Override
  public void end(boolean interrupted)
  {
    shooter.stop();
    kicker.stopKicker();
    indexer.stopIndexer();
  }

  @Override
  public boolean isFinished()
  {
    return false;
  }

  private record RecordedShot(Distance distance, AngularVelocity shooterSpeed, Time tof)
  {

    public Pair<Double, Double> getRPM() {return Pair.of(distance.in(Meters), shooterSpeed.in(RPM));}

    public Pair<Double, Double> getTOF() {return Pair.of(distance.in(Meters), tof.in(Second));}
  }
}