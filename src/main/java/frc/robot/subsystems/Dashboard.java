// A helper class to manage the dashboard and the values that will be displayed on it.
package frc.robot.subsystems;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Optional;

public class Dashboard
{

  public static        NetworkTable      dashboardTable         = NetworkTableInstance.getDefault().getTable("Dashboard");
  private static final BooleanSubscriber manualOverrideReciever = dashboardTable.getBooleanTopic("AutonomousOverride")
                                                                                .subscribe(false);
  private static final StringSubscriber  allianceBackupSelector = dashboardTable.getStringTopic("AllianceBackup")
                                                                                .subscribe("Blue");
  private static final BooleanPublisher  elevatorDownPublisher  = dashboardTable.getBooleanTopic("DriveUnderTrench")
                                                                               .publish();
  private static final BooleanPublisher hubActivePublisher     = dashboardTable.getBooleanTopic("HubActive").publish();
  private static final DoublePublisher  matchPhaseChangePublisher = dashboardTable.getDoubleTopic("PhaseChangeIn").publish();
  private static final Field2d          field                     = new Field2d();

  public static boolean getManualOverride()
  {
    return manualOverrideReciever.get();
  }

  public static void setElevatorStatus(boolean isDown)
  {
    elevatorDownPublisher.set(isDown);
  }

  public static void field2dInit()
  {
    SmartDashboard.putData("Dashboard/Field", field);
  }

  public static Field2d getField2d()
  {
    return field;
  }

  public static Alliance getAlliance()
  {
    Optional<Alliance> shrodingersAlliance = DriverStation.getAlliance();
    if (shrodingersAlliance.isEmpty())
    {
      if (allianceBackupSelector.get().equals("Blue"))
      {
        return Alliance.Blue;
      } else if (allianceBackupSelector.get().equals("Red"))
      {
        return Alliance.Red;
      } else
      {
        return Alliance.Blue;
      }
    }
    return shrodingersAlliance.get();
  }

  public static void matchPhaseChange()
  {
    double matchTime = DriverStation.getMatchTime();
    if (160 >= matchTime && matchTime > 140)
    {
      // Autonomous, 20 seconds until next shift
      matchPhaseChangePublisher.set(matchTime - 140);
    } else if (140 >= matchTime && matchTime > 130)
    {
      // Transition shift, 10 seconds until shift 1
      matchPhaseChangePublisher.set(matchTime - 130);
    } else if (130 >= matchTime && matchTime > 105)
    {
      // Shift 1, 25 seconds until shift 2
      matchPhaseChangePublisher.set(matchTime - 105);
    } else if (105 >= matchTime && matchTime > 80)
    {
      // Shift 2, 25 seconds until shift 3
      matchPhaseChangePublisher.set(matchTime - 80);
    } else if (80 >= matchTime && matchTime > 55)
    {
      // Shift 3, 25 seconds until shift 4
      matchPhaseChangePublisher.set(matchTime - 55);
    } else if (55 >= matchTime && matchTime > 30)
    {
      // Shift 4, 25 seconds until endgame
      matchPhaseChangePublisher.set(matchTime - 30);
    } else
    {
      // 30 seconds until game ends
      matchPhaseChangePublisher.set(matchTime);
    }
  }

  public static void isHubActive()
  {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    // If we have no alliance, we cannot be enabled, therefore no hub.
    if (alliance.isEmpty())
    {
      hubActivePublisher.set(false);
      return;
    }
    // Hub is always enabled in autonomous.
    if (DriverStation.isAutonomousEnabled())
    {
      hubActivePublisher.set(true);
      return;
    }
    // At this point, if we're not teleop enabled, there is no hub.
    if (!DriverStation.isTeleopEnabled())
    {
      hubActivePublisher.set(false);
      return;
    }

    // We're teleop enabled, compute.
    double matchTime = DriverStation.getMatchTime();

    String gameData = DriverStation.getGameSpecificMessage();
    // If we have no game data, we cannot compute, assume hub is active, as its likely early in teleop.
    if (gameData.isEmpty())
    {
      hubActivePublisher.set(true);
      return;
    }
    boolean redInactiveFirst = false;
    switch (gameData.charAt(0))
    {
      case 'R' -> redInactiveFirst = true;
      case 'B' -> redInactiveFirst = false;
      default ->
      {
        // If we have invalid game data, assume hub is active.
        hubActivePublisher.set(true);
        return;
      }
    }

    // Shift was is active for blue if red won auto, or red if blue won auto.
    boolean shift1Active = switch (alliance.get())
    {
      case Red -> !redInactiveFirst;
      case Blue -> redInactiveFirst;
    };

    if (matchTime > 130)
    {
      // Transition shift, hub is active.
      hubActivePublisher.set(true);
    } else if (matchTime > 105)
    {
      // Shift 1
      hubActivePublisher.set(shift1Active);
    } else if (matchTime > 80)
    {
      // Shift 2
      hubActivePublisher.set(!shift1Active);
    } else if (matchTime > 55)
    {
      // Shift 3
      hubActivePublisher.set(shift1Active);
    } else if (matchTime > 30)
    {
      // Shift 4
      hubActivePublisher.set(!shift1Active);
    } else
    {
      // End game, hub always active.
      hubActivePublisher.set(true);
    }
  }
}
