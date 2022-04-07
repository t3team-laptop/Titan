// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Shooter extends SubsystemBase {
  public WPI_TalonSRX shootySucky;
  public WPI_TalonFX shootyLaunchy;
  public Shooter(ShuffleBoardConfig config) {
    shootySucky = new WPI_TalonSRX(Constants.SHOOTER_SUCK_MOTOR);
    shootySucky.setInverted(true);
    shootyLaunchy = new WPI_TalonFX(Constants.SHOOTER_LAUNCH_MOTOR);
    shootyLaunchy.setInverted(true);  
    shootyLaunchy.setNeutralMode(NeutralMode.Brake);
    shootyLaunchy.configFactoryDefault();
    shootyLaunchy.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 500);
    shootyLaunchy.setSelectedSensorPosition(0);
    shootyLaunchy.config_kP(0 , config.getRPMData(1), 0);
    shootyLaunchy.config_kI(0 , 0, 0);
    shootyLaunchy.config_kD(0 , 0.01, 0);
    shootyLaunchy.config_kF(0 , 0.0495 ,0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("RPMSpeed", shootyLaunchy.getSelectedSensorVelocity()*600/2048);
  }

  //Return neo 550 Hall effect encoder

  //Runs and stops the motors
  public void shootySuckyRun(double speed){
    shootySucky.set(speed);
  }
  public void shootySuckyStop(){
    shootySucky.stopMotor();
  }

  //Runs and stops hood motor

  //Runs and stops the Flywheel 
  public void shootyLaunchyRun(double rpm){
    shootyLaunchy.set(ControlMode.Velocity, rpm/600*2048);
    //shootyLaunchy.set(Contro, value);)
  }
  public void shootyLaunchyIdle(){
    shootyLaunchy.set(Constants.SHOOTER_IDLE_SPEED);
  }
  public void shootyLaunchyStop(){
    shootyLaunchy.stopMotor();
  }


  //Returns the veloctiy of the Flywheel
  public double getShootyLaunchyRPM(){
    return shootyLaunchy.getSelectedSensorVelocity()*600/2048;
  }
}

