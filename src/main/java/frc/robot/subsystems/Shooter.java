// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  WPI_TalonSRX shootySucky;
  WPI_TalonFX shootyLaunchy;
  CANSparkMax shooterHoody;
  RelativeEncoder hoodEncoder;
  public Shooter() {
    shootySucky = new WPI_TalonSRX(Constants.SHOOTER_SUCK_MOTOR);
    shooterHoody = new CANSparkMax(Constants.SHOOTER_HOOD_PITCH,  MotorType.kBrushless);
    shootyLaunchy = new WPI_TalonFX(Constants.SHOOTER_LAUNCH_MOTOR);
    hoodEncoder = shooterHoody.getEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //Runs and stops the motors
  public void shootySuckyRun(){
    shootySucky.set(Constants.SHOOTER_SUCK_SPEED);
  }
  public void shootySuckyStop(){
    shootySucky.stopMotor();
  }

  public void shooterHoodRun(){
    shooterHoody.set(Constants.SHOOTER_HOOD_SPEED);
  }
  public void shooterHoodStop(){
    shooterHoody.stopMotor();
  }

  public void shootyLaunchyRun(){
    shootyLaunchy.set(Constants.SHOOTER_LAUNCH_SPEED);
  }
  public void shootyLaunchyIdle(){
    shootyLaunchy.set(Constants.SHOOTER_IDLE_SPEED);
  }
  public void shootyLaunchyStop(){
    shootyLaunchy.stopMotor();
  }
}
