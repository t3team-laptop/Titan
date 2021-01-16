// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */
  WPI_TalonSRX leftTop, leftFront, leftBack, rightTop, rightFront, rightBack;
  SpeedControllerGroup leftMotors, rightMotors;
  DifferentialDrive drive;
  public DriveTrain() {

  leftTop = new WPI_TalonSRX(Constants.LEFT_TOP);
  leftFront = new WPI_TalonSRX(Constants.LEFT_FRONT);
  leftFront.setInverted(true);
  leftBack = new WPI_TalonSRX(Constants.LEFT_BACK);
  leftBack.setInverted(true);
  rightTop = new WPI_TalonSRX(Constants.RIGHT_TOP);
  rightTop.setInverted(true);
  rightFront = new WPI_TalonSRX(Constants.RIGHT_FRONT);
  rightBack = new WPI_TalonSRX(Constants.RIGHT_BACK);

  leftMotors = new SpeedControllerGroup(leftTop, leftFront, leftBack);
  rightMotors = new SpeedControllerGroup(rightTop, rightFront, rightBack);
  drive = new DifferentialDrive(leftMotors, rightMotors);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void driveTankDrive(XboxController controller,double speed){
    drive.tankDrive(controller.getRawAxis(Constants.LEFT_JOY_Y)*speed, controller.getRawAxis(Constants.RIGHT_JOY_Y)*speed, true);
  }
  public void driveArcadeDrive(XboxController controller,double speed){
    drive.arcadeDrive(controller.getRawAxis(Constants.LEFT_JOY_Y)*speed, controller.getRawAxis(Constants.RIGHT_JOY_X)*speed, true);
  }
  public void driveGtaDrive(XboxController controller,double speed){
    drive.arcadeDrive(((controller.getRawAxis(Constants.RIGHT_TRIG))-(controller.getRawAxis(Constants.LEFT_TRIG)))*speed, controller.getRawAxis(Constants.LEFT_JOY_X)*speed, true);
  }
  public void stop(){
    drive.stopMotor();
  }
  public void driveForward(double speed){
    drive.tankDrive(speed, speed);
  }
}
