// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//31-32 is inverted false 33-34 is inverted true
package frc.robot;

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
//Autonomous
import frc.robot.commands.AutoCommands.AutoIntake;
import frc.robot.commands.AutoCommands.AutonomousPathOne;
import frc.robot.commands.Deprecated.DriveForwardDistance;
//Driving
import frc.robot.commands.DriveForwardTimed;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.LoadShooter;
import frc.robot.commands.LocateHoop;
import frc.robot.subsystems.DriveTrain;
import frc.robot.commands.ToggleDrive;
import frc.robot.commands.ToggleIntake;
import frc.robot.commands.runTurret;

//Miscellaneous
//import frc.robot.subsystems.Jukebox;

//Indexing
import frc.robot.subsystems.Indexing;
import frc.robot.commands.MoveIndexing;
//Intake
import frc.robot.subsystems.Intake;
import frc.robot.commands.RunIntake;
//zimport frc.robot.commands.RunJukebox;

//Shooter
import frc.robot.subsystems.Shooter;
import frc.robot.commands.ShootBall;
import frc.robot.commands.LoadShooter;

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
  private final ToggleDrive toggleDrive;
  public final DriveForwardTimed driveForwardTimed;
  public final DriveForwardDistance driveForwardDistance;
  public static XboxController driverJoystick;
  JoystickButton A, B, X, Y, LB, RB, LT, RT, M1, M2;


  //Indexing
  private final Indexing indexing;
  private final MoveIndexing moveIndexingFORWARD;

  //Intake
  private final Intake intake;
  private final RunIntake runIntakeForward;

  private final ToggleIntake toggleIntake;

  //Everything Shooting
  private final Limelight limelight;
  private final LocateHoop locateHoop;
  private final Shooter shooter;
  private final ShootBall shootBall;
  private final LoadShooter loadShooter;
  private runTurret runTurretLeft;
  private runTurret runTurretRight;

  //Music
  //private final Jukebox jukebox;
  //private final RunJukebox runJukebox;

  //Autonomous
  private final AutoIntake autoIntake;
  private final AutonomousPathOne autonomousPathOne;

  SendableChooser<Command> chooser = new SendableChooser<Command>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    driveTrain = new DriveTrain();
    driveWithJoysticks = new DriveWithJoysticks(driveTrain);
    driveWithJoysticks.addRequirements(driveTrain);
    driveTrain.setDefaultCommand(driveWithJoysticks);    
    toggleDrive = new ToggleDrive(driveTrain);
    toggleDrive.addRequirements(driveTrain);

    driveForwardTimed = new DriveForwardTimed(driveTrain);
    driveForwardTimed.addRequirements(driveTrain);
    driveForwardDistance = new DriveForwardDistance(driveTrain);
    driveForwardDistance.addRequirements(driveTrain);

    //jukebox = new Jukebox();
    //runJukebox = new RunJukebox(jukebox);
    
    driverJoystick = new XboxController(Constants.JOYSTICK_NUMBER);
    
    indexing = new Indexing();
    moveIndexingFORWARD = new MoveIndexing(indexing);
    moveIndexingFORWARD.addRequirements(indexing);

    intake = new Intake();
    runIntakeForward = new RunIntake(intake, false);
    runIntakeForward.addRequirements(intake);
    toggleIntake = new ToggleIntake(intake);
    toggleIntake.addRequirements(intake);

    limelight = new Limelight();
    locateHoop = new LocateHoop(limelight);
    locateHoop.addRequirements(limelight);

    runTurretLeft = new runTurret(limelight, true);
    runTurretRight = new runTurret(limelight, false);

    shooter = new Shooter();
    shootBall = new ShootBall(shooter, limelight);
    shootBall.addRequirements(shooter, limelight);
    loadShooter = new LoadShooter(shooter);
    loadShooter.addRequirements(shooter);

    autoIntake = new AutoIntake(indexing, intake);
    autoIntake.addRequirements(indexing, intake);
    autonomousPathOne = new AutonomousPathOne(driveTrain, indexing, intake);
    autonomousPathOne.addRequirements(driveTrain, indexing, intake);

    chooser.setDefaultOption("AutonomousPathOne", autonomousPathOne);
    SmartDashboard.putData("Autonomous", chooser);

    //Declare Joystick Buttons
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

    A.toggleWhenPressed(moveIndexingFORWARD);
    A.toggleWhenPressed(runIntakeForward);
    LB.whileHeld(runTurretLeft);
    RB.whileHeld(runTurretRight);
    //X.whenHeld(moveIntakeUp);
    //Y.whenHeld(moveIntakeDown);
    X.whenPressed(toggleIntake);
    M2.whileHeld(toggleDrive);
    //Y.whileHeld(loadShooter);
    Y.whileHeld(shootBall);
    B.whenPressed(locateHoop); //Made it so that we can toggle locating

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    //return chooser.getSelected();
    return driveForwardTimed;
  }

  public SequentialCommandGroup getAutoPath(){
    return autonomousPathOne;
  }

}