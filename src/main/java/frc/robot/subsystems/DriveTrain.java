// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.motorcontrol.*;
import edu.wpi.first.wpilibj.simulation.ADIS16470_IMUSim;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  public WPI_TalonFX leftFront;
  public WPI_TalonFX leftBack;
  public WPI_TalonFX rightFront;
  public WPI_TalonFX rightBack;
  MotorControllerGroup leftMotors;
  MotorControllerGroup rightMotors;
  DifferentialDrive drive;
  public ADIS16470_IMU gyro = new ADIS16470_IMU();

  
  /** Creates a new DriveTrain. */
  public DriveTrain() {
    leftFront = new WPI_TalonFX(Constants.LEFT_FRONT);
    leftFront.setInverted(true);
    leftBack = new WPI_TalonFX(Constants.LEFT_BACK);
    leftBack.setInverted(true);
    rightFront = new WPI_TalonFX(Constants.RIGHT_FRONT);
    rightFront.setInverted(false);
    rightBack = new WPI_TalonFX(Constants.RIGHT_BACK);
    rightBack.setInverted(false);

    leftMotors = new MotorControllerGroup(leftFront, leftBack);
    rightMotors = new MotorControllerGroup(rightFront, rightBack);
    drive = new DifferentialDrive(leftMotors, rightMotors);
  }

  @Override
  public void periodic() {
  }


  public void driveArcadeDrive(XboxController controller,double speed){
    //System.out.println(DriverStation.getMatchTime());

    drive.arcadeDrive(controller.getRawAxis(Constants.LEFT_JOY_Y)*speed, controller.getRawAxis(Constants.RIGHT_JOY_X)*0.65*-1, true);
  }

  public void stop(){
    drive.stopMotor();
  }

  public void driveArcade(double speed, double rotation){
    drive.arcadeDrive(speed, rotation);
  }
  
  public void driveForward(double speed){
    drive.tankDrive(speed, speed);
  }

  public void driveTank(double speed1, double speed2){
    drive.tankDrive(speed1, speed2);
  }

  public static int getPov(XboxController controller){
    return controller.getPOV(Constants.JOY_POV);
  }
}