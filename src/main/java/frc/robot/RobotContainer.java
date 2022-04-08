// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//31-32 is inverted false 33-34 is inverted true
package frc.robot;

import java.util.List;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
//Command and Control
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutoCommands.CenterTarget;
import frc.robot.commands.AutoCommands.AutoPeriod.AutoIntake;
import frc.robot.commands.AutoCommands.AutoPeriod.AutonomousDistanceDrive;
import frc.robot.commands.AutoCommands.AutoPeriod.AutonomousPathOne;
import frc.robot.commands.AutoCommands.AutoPeriod.AutonomousTimed;
import frc.robot.commands.AutoCommands.AutoPeriod.AutonomousTurning;
import frc.robot.commands.AutoCommands.AutoPeriod.AutonomousTwoBall;
import frc.robot.commands.Deprecated.DriveForwardDistance;
import frc.robot.commands.ManualMovements.ElevatorPull;
import frc.robot.commands.ManualMovements.MoveIndexing;
import frc.robot.commands.ManualMovements.RunIntake;
import frc.robot.commands.ManualMovements.Turret.AutoHood;
import frc.robot.commands.ManualMovements.Turret.LaunchBall;
import frc.robot.commands.ManualMovements.Turret.LoadShooter;
import frc.robot.commands.ManualMovements.Turret.ManualHood;
import frc.robot.commands.ManualMovements.Turret.ManualSpinTurret;
import frc.robot.commands.Toggles.ToggleIntake;
import frc.robot.commands.Toggles.ToggleTracking;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.subsystems.AutonomousPathDrivetrain;
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
import frc.robot.subsystems.ShuffleBoardConfig;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Hood;
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
  private final AutonomousPathDrivetrain autonomousPathDrivetrain;
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
  private final RunIntake runIntakeBackward;

  private final ToggleIntake toggleIntakeDown2;
  private final ToggleIntake toggleIntakeDown;

  //Elevator
  private final Elevator elevator;
  private final ElevatorPull elevatorPullPos;
  private final ElevatorPull elevatorPullNeg;

  //Everything Shooting
  private final Limelight limelight;
  private final Shooter shooter;
  //private final Hood hood;
  private final LaunchBall launchBallClose, launchBallMedium, launchBallDistance;
  //private final AutoHood hoodDown, hoodPos1, hoodPos2, hoodPos3;
  private final LoadShooter loadShooterForward, loadShooterBackward;
  private final ManualSpinTurret spinTurretManuelhigh;//, spinTurretManuellow;
  //private final ManualHood manualHoodUp;
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
  private final AutonomousTurning autonomousTurning;
  private final AutonomousDistanceDrive autonomousDistanceDrive;
  private final AutonomousTwoBall autonomousTwoBall;

  //private final ShuffleBoardConfig shuffleConfig;


  SendableChooser<Command> chooser = new SendableChooser<Command>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    //shuffleConfig = new ShuffleBoardConfig();

    //Initializing DriveTrain and It's Commands
    driveTrain = new DriveTrain();
    driveWithJoysticks = new DriveWithJoysticks(driveTrain);
    driveWithJoysticks.addRequirements(driveTrain);
    driveTrain.setDefaultCommand(driveWithJoysticks);
    driveForwardDistance = new DriveForwardDistance(driveTrain);
    driveForwardDistance.addRequirements(driveTrain);

    autonomousPathDrivetrain = new AutonomousPathDrivetrain();

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
    runIntakeForward = new RunIntake(intake, true);
    runIntakeForward.addRequirements(intake);
    runIntakeBackward = new RunIntake(intake, false);
    runIntakeBackward.addRequirements(intake);
    toggleIntakeDown2 = new ToggleIntake(intakeMove, true, false);
    toggleIntakeDown2.addRequirements(intakeMove);
    toggleIntakeDown = new ToggleIntake(intakeMove, false, false);
    toggleIntakeDown.addRequirements(intakeMove);
    intakeMove.setDefaultCommand(toggleIntakeDown);

    //Initializing Limelight
    limelight = new Limelight();

    //Initializing turret
    //turret = new Turret(shuffleConfig);
    turret = new Turret();
    centerTarget = new CenterTarget(turret, limelight);
    centerTarget.addRequirements(turret, limelight);
    turret.setDefaultCommand(centerTarget);

    toggleTracking = new ToggleTracking(turret);
    toggleTracking.addRequirements(turret);

    spinTurretManuelhigh = new ManualSpinTurret(turret, shooterJoystick, 0.1, true);
    //spinTurretManuellow = new ManualSpinTurret(turret, shooterJoystick, 0.1, false);

    //Intializing Shooter
    //hood = new Hood();
    //shooter = new Shooter(shuffleConfig);
    shooter = new Shooter();
    launchBallClose = new LaunchBall(shooter, limelight, Constants.SHOOTER_LAUNCH_SPEED_CLOSE);
    launchBallClose.addRequirements(shooter, limelight);
    launchBallMedium = new LaunchBall(shooter, limelight, Constants.SHOOTER_LAUNCH_SPEED_MEDIUM); // Change as necessary
    launchBallMedium.addRequirements(shooter, limelight);
    launchBallDistance = new LaunchBall(shooter, limelight, Constants.SHOOTER_LAUNCH_SPEED_DISTANCE);
    launchBallDistance.addRequirements(shooter, limelight);
    //launchBallDistance = new LaunchBall(shooter, limelight, 0, true, shuffleConfig); // Change as necessary
    //launchBallDistance.addRequirements(shooter, limelight, shuffleConfig);
    loadShooterForward = new LoadShooter(shooter, true);
    loadShooterForward.addRequirements(shooter);
    loadShooterBackward = new LoadShooter(shooter, false);
    loadShooterBackward.addRequirements(shooter);
    // hoodDown = new AutoHood(hood, limelight, 0);
    // hoodPos1 = new AutoHood(hood, limelight, 1);
    // hoodPos2 = new AutoHood(hood, limelight, 2);
    // hoodPos3 = new AutoHood(hood, limelight, 3);
    // hoodDown.addRequirements(hood, limelight);
    // hoodPos1.addRequirements(hood, limelight);
    // hoodPos2.addRequirements(hood, limelight);
    //hoodPos3.addRequirements(hood, limelight);
    //manualHoodUp = new ManualHood(hood, true, limelight);
    //manualHoodUp.addRequirements(hood, limelight);

    //Initializing Climber
    elevator = new Elevator();
    elevatorPullPos = new ElevatorPull(elevator, true);
    elevatorPullPos.addRequirements(elevator);
    elevatorPullNeg = new ElevatorPull(elevator, false);
    elevatorPullNeg.addRequirements(elevator);

    //Initializing Autonomous Code
    autonomousTimed = new AutonomousTimed(driveTrain, shooter, launchBallMedium);    
    autonomousTimed.addRequirements(driveTrain, shooter);
    autoIntake = new AutoIntake(indexing, intake, intakeMove);
    autoIntake.addRequirements(indexing, intake);
    autonomousPathOne = new AutonomousPathOne(driveTrain, indexing, intake);
    autonomousPathOne.addRequirements(driveTrain, indexing, intake);

    autonomousTwoBall = new AutonomousTwoBall(driveTrain, indexing, intakeMove, intake, shooter, turret, autonomousPathDrivetrain, limelight);
    autonomousTwoBall.addRequirements(driveTrain, indexing, intakeMove, intake, shooter, turret, autonomousPathDrivetrain, limelight);

    autonomousTurning = new AutonomousTurning(autonomousPathDrivetrain, driveTrain, 90);
    autonomousTurning.addRequirements(driveTrain);
    autonomousDistanceDrive = new AutonomousDistanceDrive(driveTrain, 60);
    autonomousDistanceDrive.addRequirements(driveTrain);

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
    RB.whileHeld(toggleIntakeDown2);
    RB.whileHeld(moveIndexingFORWARD);
    RB.whileHeld(runIntakeForward);
    LB.whileHeld(moveIndexingFORWARD);
    LB.whileHeld(loadShooterForward);
    M1.whileHeld(loadShooterBackward); //it works!!
    Y.toggleWhenPressed(launchBallMedium);
    X.whenPressed(toggleTracking);
    //X.whenPressed(autonomousTurning);
    //A.whileHeld(manualHoodUp);
    
    //Configure Shooter Controller Buttons
    SX.toggleWhenPressed(launchBallClose);
    SA.toggleWhenPressed(launchBallMedium);
    SB.toggleWhenPressed(launchBallDistance);
    SM1.toggleWhenPressed(spinTurretManuelhigh);
    SRB.whileHeld(elevatorPullPos); 
    SLB.whileHeld(elevatorPullNeg);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                Constants.ksVolts,
                Constants.kvVoltSecondsPerMeter,
                Constants.kaVoltSecondsSquaredPerMeter),
            Constants.kDriveKinematics,
            10);

    TrajectoryConfig config =
        new TrajectoryConfig(
              Constants.kMaxSpeedMetersPerSecond,
              Constants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);    
            
    // An example trajectory to follow.  All units in meters.
    // Pretty sure since we are using PathPlanner that this is unnecessary so uncoment later
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            // Pass config
            config);

    Trajectory pathplanner = PathPlanner.loadPath("5 Ball", 4, 3);

    RamseteCommand ramseteCommand =
        new RamseteCommand(
            pathplanner,
            autonomousPathDrivetrain::getPose,
            new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
            new SimpleMotorFeedforward(
                Constants.ksVolts,
                Constants.kvVoltSecondsPerMeter,
                Constants.kaVoltSecondsSquaredPerMeter),
            Constants.kDriveKinematics,
            autonomousPathDrivetrain::getWheelSpeeds,
            new PIDController(Constants.kPDriveVel, 0, 0),
            new PIDController(Constants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            autonomousPathDrivetrain::tankDriveVolts,
            autonomousPathDrivetrain);
    
    // Reset odometry to the starting pose of the trajectory.
    autonomousPathDrivetrain.resetOdometry(exampleTrajectory.getInitialPose());
    
    // Run path following command, then stop at the end.
    //return ramseteCommand.andThen(() -> autonomousPathDrivetrain.tankDriveVolts(0, 0));

    //return chooser.getSelected();
    //return autonomousTimed;
    //return autonomousTurning;
    return autonomousTwoBall;
  }

  public SequentialCommandGroup getAutoPath(){
    return autonomousPathOne;
  }

  public DriveTrain gDriveTrain(){
    return driveTrain;
  }

}