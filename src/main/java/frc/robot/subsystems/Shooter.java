// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Shooter extends SubsystemBase {
  public WPI_TalonSRX shootySucky;
  public WPI_TalonFX shootyLaunchy;
  public CANSparkMax shooterHood;
  public RelativeEncoder hoodEncoder;
  public Shooter() {
    shootySucky = new WPI_TalonSRX(Constants.SHOOTER_SUCK_MOTOR);
    shootySucky.setInverted(true);
    shooterHood = new CANSparkMax(Constants.SHOOTER_HOOD_PITCH,  MotorType.kBrushless);
    //hoodEncoder = shooterHood.getEncoder();
    shootyLaunchy = new WPI_TalonFX(Constants.SHOOTER_LAUNCH_MOTOR);
    shootyLaunchy.setInverted(true);  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public RelativeEncoder getHoodEncoder(){
    return shooterHood.getEncoder();
  }

  //Runs and stops the motors
  public void shootySuckyRun(double speed){
    shootySucky.set(speed);
  }
  public void shootySuckyStop(){
    shootySucky.stopMotor();
  }

  public void shooterHoodRun(double speed){
    shooterHood.set(speed);
  }
  public void shooterHoodStop(){
    shooterHood.stopMotor();
  }

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
}
