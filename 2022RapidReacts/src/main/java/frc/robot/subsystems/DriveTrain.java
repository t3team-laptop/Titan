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
  public WPI_TalonFX leftFront;
  public WPI_TalonFX leftBack;
  public WPI_TalonFX rightFront;
  public WPI_TalonFX rightBack;
  MotorControllerGroup leftMotors;
  MotorControllerGroup rightMotors;
  DifferentialDrive drive;
  private static boolean tank, arcade, gta, superDrive;

  
  /** Creates a new DriveTrain. */
  public DriveTrain() {
    tank = false;
    arcade = false;
    gta = false;
    superDrive = true;
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
    // This method will be called once per scheduler run
  }

  public void driveArcadeDrive(XboxController controller,double speed){
    drive.arcadeDrive(controller.getRightX()*speed, controller.getLeftY()*speed*-1, true);
  }

  public void driveTankDrive(XboxController controller,double speed){
    drive.tankDrive(controller.getRawAxis(Constants.LEFT_JOY_Y)*speed, controller.getRawAxis(Constants.RIGHT_JOY_Y)*speed, true);
  }

  public void driveGtaDrive(XboxController controller,double speed){
    drive.arcadeDrive(((controller.getRawAxis(Constants.RIGHT_TRIG))-(controller.getRawAxis(Constants.LEFT_TRIG)))*speed, controller.getRawAxis(Constants.LEFT_JOY_X)*speed*-1, true);
  }

  public void driveSuperDrive(XboxController controller,double speed){
    drive.arcadeDrive(controller.getRawAxis(Constants.LEFT_JOY_Y)*speed,controller.getRawAxis(Constants.RIGHT_JOY_X)*speed,false); //Does not have low sensitivity squaring for now
  }

  public void stop(){
    drive.stopMotor();
  }
  
  public void driveForward(double speed){
    drive.tankDrive(speed, speed);
  }

  public void driveAuto(double speed, double targetDistance){
    drive.tankDrive(speed, speed);
  }

  public boolean getTank(){return tank;}
  public boolean getArcade(){return arcade;}
  public boolean getGta(){return gta;}
  public boolean getSuper(){return superDrive;}

  public void setTank(){ 
    tank = true;
    arcade = false;
    gta = false;
    superDrive = false;
  }

  public void setArcade(){
    arcade = true;
    tank = false;
    gta = false;
    superDrive = false;
  }

  public void setGta(){
    gta = true;
    tank = false;
    arcade = false;
    superDrive = false;
  }
  
  public void setSuper(){
    superDrive = true;
    gta = false;
    tank = false;
    arcade = false;
  }

  public static int getPov(XboxController controller){
    return controller.getPOV(Constants.JOY_POV);
  }
}