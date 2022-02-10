// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.motorcontrol.*;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  WPI_TalonFX leftTop, leftFront, leftBack, rightTop, rightFront, rightBack;
  MotorControllerGroup leftMotors;
  MotorControllerGroup rightMotors;
  DifferentialDrive drive;

  
  /** Creates a new DriveTrain. */
  public DriveTrain() {
    leftFront = new WPI_TalonFX(Constants.LEFT_FRONT);
    leftFront.setInverted(false);
    leftBack = new WPI_TalonFX(Constants.LEFT_BACK);
    leftBack.setInverted(false);
    rightFront = new WPI_TalonFX(Constants.RIGHT_FRONT);
    rightFront.setInverted(true);
    rightBack = new WPI_TalonFX(Constants.RIGHT_BACK);
    rightBack.setInverted(true);

    leftMotors = new MotorControllerGroup(leftFront, leftBack);
    rightMotors = new MotorControllerGroup( rightFront, rightBack);
    drive = new DifferentialDrive(leftMotors, rightMotors);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void driveTankDrive(XboxController controller,double speed){
    drive.tankDrive(controller.getLeftY()*speed, controller.getRightY()*speed, true);
  }

  public void stop(){
    drive.stopMotor();
  }
  
  public void driveForward(double speed){
    drive.tankDrive(speed, speed);
  }

  public void driveAuto(double speed1, double speed2){
    drive.tankDrive(speed1, speed2);
  }
}
