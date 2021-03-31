// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */
  private WPI_TalonFX leftTop, leftFront, leftBack, rightTop, rightFront, rightBack;
  private SpeedControllerGroup leftMotors, rightMotors;
  private DifferentialDrive drive;
  private static boolean tank, arcade, gta;
  public DriveTrain() {
    tank = false;
    arcade = true;
    gta = false;

  leftTop = new WPI_TalonFX(Constants.LEFT_TOP);
  leftFront = new WPI_TalonFX(Constants.LEFT_FRONT);
  leftFront.setInverted(true);
  leftBack = new WPI_TalonFX(Constants.LEFT_BACK);
  leftBack.setInverted(true);
  rightTop = new WPI_TalonFX(Constants.RIGHT_TOP);
  rightFront = new WPI_TalonFX(Constants.RIGHT_FRONT);
  rightFront.setInverted(true);
  rightBack = new WPI_TalonFX(Constants.RIGHT_BACK);
  rightBack.setInverted(true);

  leftMotors = new SpeedControllerGroup(leftTop, leftFront, leftBack);
  rightMotors = new SpeedControllerGroup(rightTop, rightFront, rightBack);
  drive = new DifferentialDrive(leftMotors, rightMotors);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //idk i saved which triggers do what in the notes app but it got deleted which is quite frustrating
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
  public void driveAuto(double speed1, double speed2){
    drive.tankDrive(speed1, speed2);
  }
  public boolean getTank(){return tank;}
  public boolean getArcade(){return arcade;}
  public boolean getGta(){return gta;}
  public void setTank(){ 
    tank = true;
    arcade = false;
    gta = false;
  }
  public void setArcade(){
    arcade = true;
    tank = false;
    gta = false;
  }
  public void setGta(){
    gta = true;
    tank = false;
    arcade = false;
  }
  public int getPov(XboxController controller){
    return controller.getPOV(Constants.JOY_POV);
  }
}
