// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveForwardTimed;
import frc.robot.commands.DriveWithJoySticks;
import frc.robot.commands.MoveElevator;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.commands.ToggleDrive;
import edu.wpi.first.wpilibj.Compressor;
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
  private final ToggleDrive toggleDrive;
  private final Compressor c;
  private final Elevator elevator;
  private final MoveElevator moveElevUP, moveElevDOWN;

  //Declare buttons
  JoystickButton A, B, X, Y, LB, RB, RT, M1, M2;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    //Initialize DriveTrain Teleop
    driveTrain = new DriveTrain();
    driveWithJoyStick = new DriveWithJoySticks(driveTrain);
    driveWithJoyStick.addRequirements(driveTrain);
    driveTrain.setDefaultCommand(driveWithJoyStick);
    toggleDrive = new ToggleDrive(driveTrain);

    //Initialize DriveTrain Auto
    driveForwardTimed = new DriveForwardTimed(driveTrain);
    driveForwardTimed.addRequirements(driveTrain);

    //Initialize Joysticks or XboxController
    driverJoystick = new XboxController(Constants.CONTROLLER_NUMBER);

    //Initialize Elevator
    elevator = new Elevator();
    moveElevUP = new MoveElevator(elevator, true);
    moveElevDOWN = new MoveElevator(elevator, false);

    // Initialize Buttons
    A = new JoystickButton(driverJoystick, Constants.BUT_A);
    B = new JoystickButton(driverJoystick, Constants.BUT_B);
    X = new JoystickButton(driverJoystick, Constants.BUT_X);
    Y = new JoystickButton(driverJoystick, Constants.BUT_Y);
    LB = new JoystickButton(driverJoystick, Constants.BUT_LB);
    RB = new JoystickButton(driverJoystick, Constants.BUT_RB);
    RT = new JoystickButton(driverJoystick, Constants.RIGHT_TRIG);
    M1 = new JoystickButton(driverJoystick, Constants.BUT_M1);
    M2 = new JoystickButton(driverJoystick, Constants.BUT_M2);

    //Compressor
    c = new Compressor(Constants.COMPRESSOR_ID);
    c.setClosedLoopControl(true);

    //Toggle commands
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
    //Y.whenPressed();
    //X.whenHeld();
    A.whenHeld(moveElevUP);
    B.whenHeld(moveElevDOWN);
    //LB.whileHeld();
    //RB.whenPressed();
    //M1.whileHeld();
    M2.whileHeld(toggleDrive);

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