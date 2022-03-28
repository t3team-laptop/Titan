// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//31-32 is inverted false 33-34 is inverted true
package frc.robot;

import edu.wpi.first.wpilibj.AnalogTrigger;
//Command and Control
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutoCommands.CenterTarget;
import frc.robot.commands.AutoCommands.AutoPeriod.AutoIntake;
import frc.robot.commands.AutoCommands.AutoPeriod.AutonomousPathOne;
import frc.robot.commands.AutoCommands.AutoPeriod.AutonomousTimed;
import frc.robot.commands.Deprecated.DriveForwardDistance;
import frc.robot.commands.ManualMovements.ElevatorPull;
import frc.robot.commands.ManualMovements.MoveIndexing;
import frc.robot.commands.ManualMovements.RunIntake;
import frc.robot.commands.ManualMovements.Turret.AdjustHood;
import frc.robot.commands.ManualMovements.Turret.LaunchBall;
import frc.robot.commands.ManualMovements.Turret.LoadShooter;
import frc.robot.commands.ManualMovements.Turret.ManualHood;
import frc.robot.commands.ManualMovements.Turret.ManualSpinTurret;
import frc.robot.commands.Toggles.ToggleIntake;
import frc.robot.commands.Toggles.ToggleTracking;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.subsystems.DriveTrain;

//Miscellaneous
//import frc.robot.subsystems.Jukebox;

//Indexing
import frc.robot.subsystems.Indexing;
//Intake
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeMove;
//Shooter
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
//Elevator
import frc.robot.subsystems.Elevator;
//Limelight
import frc.robot.subsystems.Limelight;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveTrain driveTrain;
  private final DriveWithJoysticks driveWithJoysticks;
  public final DriveForwardDistance driveForwardDistance;
  public static XboxController driverJoystick;
  public static XboxController shooterJoystick;
  JoystickButton A, B, X, Y, LB, RB, RT, LT, M1, M2;
  //AnalogTrigger LT, RT, SLT, SRT;
  JoystickButton SA, SB, SX, SY, SLB, SRB, SLT, SRT, SM1, SM2;


  //Indexing
  private final Indexing indexing;
  private final MoveIndexing moveIndexingFORWARD;

  //Intake
  private final Intake intake;
  private final IntakeMove intakeMove;
  private final RunIntake runIntakeForward;

  private final ToggleIntake toggleIntakeUp;
  private final ToggleIntake toggleIntakeDown;

  //Elevator
  private final Elevator elevator;
  private final ElevatorPull elevatorPullPos;
  private final ElevatorPull elevatorPullNeg;

  //Everything Shooting
  private final Limelight limelight;
  private final Shooter shooter;
  private final LaunchBall launchBallTarmac, launchBallHub, launchBallDistance;
  private final AdjustHood adjustHood;
  private final LoadShooter loadShooter;
  private final ManualSpinTurret runTurretLeft;
  private final ManualSpinTurret runTurretRight;
  private final ManualHood manualHoodUp;
  private final ManualHood manualHoodDown;
  private final CenterTarget centerTarget;

  private final Turret turret;

  private final ToggleTracking toggleTracking;

  //Music
  //private final Jukebox jukebox;
  //private final RunJukebox runJukebox;

  //Autonomous
  private final AutoIntake autoIntake;
  private final AutonomousPathOne autonomousPathOne;
  private final AutonomousTimed autonomousTimed;


  SendableChooser<Command> chooser = new SendableChooser<Command>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    //Initializing DriveTrain and It's Commands
    driveTrain = new DriveTrain();
    driveWithJoysticks = new DriveWithJoysticks(driveTrain);
    driveWithJoysticks.addRequirements(driveTrain);
    driveTrain.setDefaultCommand(driveWithJoysticks);
    driveForwardDistance = new DriveForwardDistance(driveTrain);
    driveForwardDistance.addRequirements(driveTrain);

    //jukebox = new Jukebox();
    //runJukebox = new RunJukebox(jukebox);
    
    //Initializing Controllers
    driverJoystick = new XboxController(Constants.JOYSTICK_NUMBER);
    shooterJoystick = new XboxController(Constants.SHOOTER_JOYSTICK_NUMBER);
    
    //Initializing Ball MoveMent Systems
    indexing = new Indexing();
    moveIndexingFORWARD = new MoveIndexing(indexing);
    moveIndexingFORWARD.addRequirements(indexing);

    intake = new Intake();
    intakeMove = new IntakeMove();
    runIntakeForward = new RunIntake(intake, false);
    runIntakeForward.addRequirements(intake);
    toggleIntakeUp = new ToggleIntake(intakeMove, true);
    toggleIntakeUp.addRequirements(intakeMove);
    toggleIntakeDown = new ToggleIntake(intakeMove, false);
    toggleIntakeDown.addRequirements(intakeMove);

    //Initializing Limelight
    limelight = new Limelight();

    //Initializing turret
    turret = new Turret();
    centerTarget = new CenterTarget(turret, limelight);
    centerTarget.addRequirements(turret, limelight);
    turret.setDefaultCommand(centerTarget);

    toggleTracking = new ToggleTracking(turret);
    toggleTracking.addRequirements(turret);

    runTurretLeft = new ManualSpinTurret(turret, true);
    runTurretRight = new ManualSpinTurret(turret, false);

    //Intializing Shooter
    shooter = new Shooter();
    launchBallTarmac = new LaunchBall(shooter, limelight, Constants.SHOOTER_LAUNCH_SPEED_TARMAC);
    launchBallTarmac.addRequirements(shooter, limelight);
    launchBallHub = new LaunchBall(shooter, limelight, Constants.SHOOTER_LAUNCH_SPEED_HUB); // Change as necessary
    launchBallHub.addRequirements(shooter, limelight);
    launchBallDistance = new LaunchBall(shooter, limelight, Constants.SHOOTER_LAUNCH_SPEED_DISTANCE); // Change as necessary
    launchBallDistance.addRequirements(shooter, limelight);
    loadShooter = new LoadShooter(shooter);
    loadShooter.addRequirements(shooter);
    adjustHood = new AdjustHood(shooter, limelight);
    adjustHood.addRequirements(shooter, limelight);
    manualHoodUp = new ManualHood(shooter, true, limelight);
    manualHoodDown = new ManualHood(shooter, false, limelight);

    //Initializing Climber
    elevator = new Elevator();
    elevatorPullPos = new ElevatorPull(elevator, true);
    elevatorPullPos.addRequirements(elevator);
    elevatorPullNeg = new ElevatorPull(elevator, false);
    elevatorPullNeg.addRequirements(elevator);

    //Initializing Autonomous Code
    autonomousTimed = new AutonomousTimed(driveTrain, shooter, launchBallTarmac);    
    autonomousTimed.addRequirements(driveTrain, shooter);
    autoIntake = new AutoIntake(indexing, intake, intakeMove);
    autoIntake.addRequirements(indexing, intake);
    autonomousPathOne = new AutonomousPathOne(driveTrain, indexing, intake);
    autonomousPathOne.addRequirements(driveTrain, indexing, intake);

    chooser.setDefaultOption("AutonomousPathOne", autonomousPathOne);
    SmartDashboard.putData("Autonomous", chooser);

    //Declare Driver Controller Buttons
    A = new JoystickButton(driverJoystick, Constants.BUT_A);
    B = new JoystickButton(driverJoystick, Constants.BUT_B);
    X = new JoystickButton(driverJoystick, Constants.BUT_X);
    Y = new JoystickButton(driverJoystick, Constants.BUT_Y);
    LB = new JoystickButton(driverJoystick, Constants.BUT_LB);
    RB = new JoystickButton(driverJoystick, Constants.BUT_RB);
    LT = new JoystickButton(driverJoystick, Constants.LEFT_TRIG);
    RT = new JoystickButton(driverJoystick, Constants.RIGHT_TRIG);
    M1 = new JoystickButton(driverJoystick, Constants.BUT_M1);
    M2 = new JoystickButton(driverJoystick, Constants.BUT_M2);

    //Declare Shooter Controller Buttons
    SA = new JoystickButton(shooterJoystick, Constants.BUT_A);
    SB = new JoystickButton(shooterJoystick, Constants.BUT_B);
    SX = new JoystickButton(shooterJoystick, Constants.BUT_X);
    SY = new JoystickButton(shooterJoystick, Constants.BUT_Y);
    SLB = new JoystickButton(shooterJoystick, Constants.BUT_LB);
    SRB = new JoystickButton(shooterJoystick, Constants.BUT_RB);
    SLT = new JoystickButton(driverJoystick, Constants.LEFT_TRIG);
    SRT = new JoystickButton(driverJoystick, Constants.RIGHT_TRIG);
    SM1 = new JoystickButton(shooterJoystick, Constants.BUT_M1);
    SM2 = new JoystickButton(shooterJoystick, Constants.BUT_M2);

    //Start jukebox
    //jukebox.startJukebox();

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //Configure Driver Controller Buttons
    RB.whileHeld(toggleIntakeDown);
    RB.whileHeld(moveIndexingFORWARD);
    RB.whileHeld(runIntakeForward);
    LB.whileHeld(moveIndexingFORWARD);
    LB.whileHeld(loadShooter);
    Y.toggleWhenPressed(launchBallTarmac);
    M2.whileHeld(toggleIntakeUp);
    B.whileHeld(runTurretLeft);
    X.whileHeld(runTurretRight);
    A.whenPressed(toggleTracking);
    
    //Configure Shooter Controller Buttons
    //Depending on what Ty wants, maybe add all shooting controls here including launching
    //SX.toggleWhenPressed(centerTarget);
    SRB.whileHeld(runTurretRight);
    SLB.whileHeld(runTurretLeft);
    SX.toggleWhenPressed(launchBallHub);
    SY.toggleWhenPressed(launchBallTarmac);
    SB.toggleWhenPressed(launchBallDistance);
    SM1.whileHeld(elevatorPullPos); 
    SM2.whileHeld(elevatorPullNeg); 
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    //return chooser.getSelected();
    return autonomousTimed;
  }

  public SequentialCommandGroup getAutoPath(){
    return autonomousPathOne;
  }

}