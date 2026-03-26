// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import static edu.wpi.first.units.Units.Degrees;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Autonomous.Auto;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoAimCommand;
import frc.robot.commands.IntakeToggleCommand;
import frc.robot.commands.ShakeIntake;
import frc.robot.commands.ShootKickIndexCommand;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSpinSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed

  private static final double                   TRIGGER_DEADBAND = 0.1;
  final CommandXboxController driverXbox = new CommandXboxController(0);
  // The robot's subsystems and commands are defined here...
  private final        SwerveSubsystem          drivebase        = new SwerveSubsystem(driverXbox);
  SwerveInputStream driveRobotOriented = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                              () -> driverXbox.getLeftY() * 0.7,
                                                              () -> driverXbox.getLeftX() * 0.7)
                                                          .withControllerRotationAxis(() -> driverXbox.getRawAxis(
                                                              4) * -0.7)
                                                          .deadband(OperatorConstants.DEADBAND)
                                                          .scaleTranslation(0.6)
                                                          .robotRelative(true);
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -1,
                                                                () -> driverXbox.getLeftX() * -1)
                                                            .withControllerRotationAxis(() -> driverXbox.getRawAxis(
                                                                4) * -1)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(false);
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(true);
  private final        IntakeSubsystem          Intake           = new IntakeSubsystem();
  private final        ShooterSubsystem         Shooter          = new ShooterSubsystem();
  private final        KickerSubsystem          Kicker           = new KickerSubsystem();
  private final        IndexerSubsystem         Indexer          = new IndexerSubsystem();
  //private final ClimberSubsystem               Climber    = new ClimberSubsystem();   SC..SC...SCRAPPEDDDDD
  private final        IntakeSpinSubsystem      IntakeSpin       = new IntakeSpinSubsystem();
  private final        CameraSubsystem          cameraSubsystem  = new CameraSubsystem();  
  // Establish a Sendable Chooser that will be able to be sent to the SmartDashboard, allowing selection of desired auto
  private final        SendableChooser<Command> autoChooser      = new SendableChooser<>();
  private final        Auto                     m_auto           = new Auto(drivebase, Shooter, Intake);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */

  private final Trigger rightTriggerDeadband =
      new Trigger(() -> driverXbox.getRightTriggerAxis() > TRIGGER_DEADBAND); 
     //These are lowkey useless but its just a trigger with a deaband
  private final Trigger leftTriggerDeadband =
      new Trigger(() -> driverXbox.getLeftTriggerAxis() > TRIGGER_DEADBAND);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */


  public RobotContainer()
  {

    // NamedCommands.registerCommand("DriveToLimelight", drivebase.driveToLimelightTarget());
    // Configure the trigger bindings
    configureBindings();
    defaultCommands();
    DriverStation.silenceJoystickConnectionWarning(true);

    //Set up auto commands

    NamedCommands.registerCommand("INTAKEDOWN", Intake.setAngle(Degrees.of(135)).withTimeout(5));
    NamedCommands.registerCommand("INTAKEUP", Intake.setAngle(Degrees.of(0)));
    NamedCommands.registerCommand("INTAKE", IntakeSpin.runIntakeCommand(1).withTimeout(5));
    NamedCommands.registerCommand("NOSHOOT", new ShootKickIndexCommand(Shooter, Kicker, Indexer, 0).withTimeout(2));
    NamedCommands.registerCommand("STOPINTAKE", IntakeSpin.runIntakeCommand(0));
    NamedCommands.registerCommand("NEARSHOOT", new ShootKickIndexCommand(Shooter, Kicker, Indexer, 3450).withTimeout(12));
    NamedCommands.registerCommand("FARSHOOT", new ShootKickIndexCommand(Shooter, Kicker, Indexer, 3900).withTimeout(12));
    NamedCommands.registerCommand("AUTOAIM",new AutoAimCommand(drivebase));
    NamedCommands.registerCommand("AUTOSHOOT",new ShootKickIndexCommand(Shooter, Kicker, Indexer,drivebase).withTimeout(12));
    NamedCommands.registerCommand("INTAKEONANDDOWN",new IntakeToggleCommand(Intake/* ,IntakeSpin*/).withTimeout(1));
    NamedCommands.registerCommand("SHAKE", ShakeIntake.shake(Intake));
    NamedCommands.registerCommand("ShootIndexKick",
                                  new ShootKickIndexCommand(Shooter, Kicker, Indexer, 4000).withTimeout(10));
    drivebase.setupPathPlanner();

    //Set the default auto (do nothing)
    autoChooser.setDefaultOption("Do Nothing", Commands.none());
    autoChooser.addOption("LeftBumpAutoBlue", AutoBuilder.buildAuto("LeftBumpAutoBlue"));
    autoChooser.addOption("RightBumpAutoBlue", AutoBuilder.buildAuto("RightBumpAutoBlue"));
    autoChooser.addOption("StraightAutoBLUE", AutoBuilder.buildAuto("StraigtAutoBLUE"));
    autoChooser.addOption("DepotBlue", AutoBuilder.buildAuto("DepotBLUE"));
    autoChooser.addOption("CenterAutoLeftBumpBlue", AutoBuilder.buildAuto("BlueOverLeftBump"));
    autoChooser.addOption("Drive Forward", drivebase.driveForward().withTimeout(1)); //BASIC DRIVE FORWARD
    //Put the autoChooser on the SmartDashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);


  }

  //Setting default commands
  public void defaultCommands()
  {
   IntakeSpin.setDefaultCommand(IntakeSpin.run(() -> {}));
    Shooter.setDefaultCommand(Shooter.setDutyCycle(0));
    Indexer.setDefaultCommand(Indexer.stopIndexerCommand());
    Kicker.setDefaultCommand(Kicker.runKickerCommand(0));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */


  private void configureBindings()
  {
    //Add this back to make it robot Oriented
    drivebase.setDefaultCommand(
        drivebase.driveWithChassisSpeedsSupplier(
            drivebase.buildRelativeTurningStream()
                                                )
                               );

    // .onTrue(Commands.runOnce(() -> drivebase.setDriveToPoseActive(true)))
    // .onFalse(Commands.runOnce(() -> drivebase.setDriveToPoseActive(false)));

// LIMELIGHT CONTROLS
//driverXbox.rightBumper().whileTrue(drivebase.driveToLimelightTarget());

    // INTAKE CONTROLS

    //leftTriggerDeadband.toggleOnTrue(Intake.setAngle(Degrees.of(120)));
    //leftTriggerDeadband.toggleOnFalse(Intake.setAngle(Degrees.of(0)));

driverXbox.povDown().whileTrue(Indexer.runIndexerCommand(-1).alongWith(Kicker.runKickerCommand(1))); //backup incase shooter encoders break
driverXbox.povUp().whileTrue(Indexer.runIndexerCommand(-1).alongWith(Kicker.runKickerCommand(1)));

// DUTY CYCLE INTAKE INCASE SETPOINTS BREAK
driverXbox.a().whileTrue(Intake.armCmd(0.3));
driverXbox.a().whileFalse(Intake.armCmd(0));
driverXbox.y().whileTrue(Intake.armCmd(-0.3));
driverXbox.y().whileFalse(Intake.armCmd(0));


//INTAKE COMMAND
driverXbox.leftBumper().onTrue(new IntakeToggleCommand(Intake/* ,IntakeSpin*/));
driverXbox.leftTrigger().whileTrue(IntakeSpin.runIntakeCommand(1));
driverXbox.leftTrigger().whileTrue(IntakeSpin.runIntakeCommand(0));
//AUTO AIM TO HUB COMMAND
    driverXbox.rightBumper().whileTrue(new AutoAimCommand(drivebase));

//SHOOTER KICKER INDEXER CONTROLS
//AUTO RPM SHOOT
    driverXbox.rightTrigger(0.2).whileTrue(new ShootKickIndexCommand(Shooter, Kicker, Indexer, drivebase));
   
    //SHOOT FROM TOWER
    driverXbox.x().whileTrue(new ShootKickIndexCommand(Shooter,
                                                                     Kicker,
                                                                     Indexer,
                                                                     Constants.ShooterConstants.FARShooterGoalRPM));
    //SHOOT FROM CLOSE
    driverXbox.b().whileTrue(new ShootKickIndexCommand(Shooter,
                                                       Kicker,
                                                       Indexer,
                                                       Constants.ShooterConstants.NEARShooterGoalRPM));

                                            
//RESET HYRO FOR FOD
    driverXbox.start().and(driverXbox.back()).onTrue(drivebase.zeroGyroWtihAlliance());


    //SHAKE COMMAND 
    driverXbox.pov(270).onTrue(ShakeIntake.shake(Intake));


  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand()
  {
    // Pass in the selected auto from the SmartDashboard as our desired autnomous commmand 
    //return autoChooser.getSelected();
   // return drivebase.getAutonomousCommand("DepotBLUE");   // CHANGE THIS TO CHANGE THE AUTO
     return autoChooser.getSelected();
  }

}