// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutoAim;
import frc.robot.commands.DriveForwardTimed;
import frc.robot.commands.DriveWithJoySticks;
import frc.robot.subsystems.BallSuck;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Uptake;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootBall;
import frc.robot.commands.ShooterState;
import frc.robot.commands.ToggleShooter;
import frc.robot.commands.UptakeCommand;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 *Daniel: make intake have A button
 uptake has B and Y (back and forth)
 *
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  //Declare DriveTrain
  private final DriveTrain driveTrain;
  private final DriveWithJoySticks driveWithJoyStick;
  private final DriveForwardTimed driveForwardTimed;
  public static XboxController driverJoystick;
  private final Intake intake;
  private final IntakeCommand intakeCommand;
  private final Uptake uptake;
  private final UptakeCommand uptakeCommand1, uptakeCommand2;
  private final Shooter shooty;
  private final ShooterState shooterState;
  private final Limelight limy;
  private final BallSuck ballsuccy;
  private final ShootBall shootBall;
  private final ToggleShooter toggleShooter;
  private final AutoAim autoAim; 

  //Declare buttons
  JoystickButton A, B, X, Y, LB, RB, RT;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    //Initialize DriveTrain Teleop
    driveTrain = new DriveTrain();
    driveWithJoyStick = new DriveWithJoySticks(driveTrain);
    driveWithJoyStick.addRequirements(driveTrain);
    driveTrain.setDefaultCommand(driveWithJoyStick);

    //Initialize DriveTrain Auto
    driveForwardTimed = new DriveForwardTimed(driveTrain);
    driveForwardTimed.addRequirements(driveTrain);

    //Initialize Ballsuck
    ballsuccy = new BallSuck();
    shootBall = new ShootBall(ballsuccy);

    //Initialize Limelight
    limy = new Limelight();
    autoAim = new AutoAim(limy, driveTrain);


    //Initialize Shooter Teleop
    shooty = new Shooter();
    shooterState = new ShooterState(shooty, limy);
    toggleShooter = new ToggleShooter();

    //Initialize Joysticks or XboxController
    driverJoystick = new XboxController(Constants.CONTROLLER_NUMBER);
    
    // Initialize Buttons
    A = new JoystickButton(driverJoystick, Constants.BUT_A);
    B = new JoystickButton(driverJoystick, Constants.BUT_B);
    X = new JoystickButton(driverJoystick, Constants.BUT_X);
    Y = new JoystickButton(driverJoystick, Constants.BUT_Y);
    LB = new JoystickButton(driverJoystick, Constants.BUT_LB);
    RB = new JoystickButton(driverJoystick, Constants.BUT_RB);
    RT = new JoystickButton(driverJoystick, Constants.RIGHT_TRIG);

    // Initialize Intake
    intake = new Intake();
    intakeCommand = new IntakeCommand(intake);
    intakeCommand.addRequirements(intake);

    // Initialize Uptake
    uptake = new Uptake();
    uptakeCommand1 = new UptakeCommand(uptake, false);
    uptakeCommand2 = new UptakeCommand(uptake, true);
    uptakeCommand1.addRequirements(uptake);
    uptakeCommand2.addRequirements(uptake);
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
    Y.whenPressed(autoAim);
    X.whenHeld(intakeCommand);
    A.whenHeld(uptakeCommand1);
    B.whenHeld(uptakeCommand2);
    RB.whenPressed(toggleShooter);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return driveForwardTimed;
  }
}
