// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import java.lang.AutoCloseable;


public class Shooter extends SubsystemBase {
  public WPI_TalonSRX shootySucky;
  public WPI_TalonFX shootyLaunchy;
  public CANSparkMax shooterHood;
  public Shooter() {
    shootySucky = new WPI_TalonSRX(Constants.SHOOTER_SUCK_MOTOR);
    shooterHood = new CANSparkMax(Constants.SHOOTER_HOOD_PITCH,  MotorType.kBrushless);
    shootyLaunchy = new WPI_TalonFX(Constants.SHOOTER_LAUNCH_MOTOR);
    shootyLaunchy.setInverted(true);
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
    shooterHood.set(Constants.SHOOTER_HOOD_SPEED);
  }
  public void shooterHoodStop(){
    shooterHood.stopMotor();
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
