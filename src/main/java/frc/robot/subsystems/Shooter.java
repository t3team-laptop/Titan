// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Shooter extends SubsystemBase {
  public WPI_TalonSRX shootySucky;
  public WPI_TalonFX shootyLaunchy;
  public CANSparkMax shooterHood;
  public RelativeEncoder hoodEncoder;
  public SparkMaxPIDController hoodPidController;
  public Shooter() {
    shootySucky = new WPI_TalonSRX(Constants.SHOOTER_SUCK_MOTOR);
    shootySucky.setInverted(true);
    shooterHood = new CANSparkMax(Constants.SHOOTER_HOOD_PITCH,  MotorType.kBrushless);
    hoodEncoder = shooterHood.getEncoder();
    shootyLaunchy = new WPI_TalonFX(Constants.SHOOTER_LAUNCH_MOTOR);
    shootyLaunchy.setInverted(true);  
    hoodPidController = shooterHood.getPIDController();
    hoodPidController.setFeedbackDevice(hoodEncoder);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Hood Encoder Position", hoodEncoder.getPosition());
  }

  //Return neo 550 Hall effect encoder
  public RelativeEncoder getHoodEncoder(){
    return hoodEncoder;
  }

  public SparkMaxPIDController getHoodPidController(){
    return hoodPidController;
  }

  //Runs and stops the motors
  public void shootySuckyRun(double speed){
    shootySucky.set(speed);
  }
  public void shootySuckyStop(){
    shootySucky.stopMotor();
  }

  //Runs and stops hood motor
  public void shooterHoodRun(double speed){
    shooterHood.set(speed);
  }
  public void shooterHoodStop(){
    shooterHood.stopMotor();
  }

  //Runs and stops the Flywheel 
  public void shootyLaunchyRun(double speed){
    shootyLaunchy.set(speed);
    //shootyLaunchy.set(Contro, value);)
  }
  public void shootyLaunchyIdle(){
    shootyLaunchy.set(Constants.SHOOTER_IDLE_SPEED);
  }
  public void shootyLaunchyStop(){
    shootyLaunchy.stopMotor();
  }

  //Returns the veloctiy of the Flywheel
  public double getShootyLaunchyVelocity(){
    return shootyLaunchy.getSelectedSensorVelocity();
  }
  public double getHoodPosition(){
    return hoodEncoder.getPosition();
  }
}
