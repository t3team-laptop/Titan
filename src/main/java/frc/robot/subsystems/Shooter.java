// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private WPI_TalonFX shooterMotor;
  /** Creates a new Shooter. */
  public Shooter() {
    shooterMotor = new WPI_TalonFX(Constants.SHOOTER_MOTOR);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void shooterRevUP(double speed){
    shooterMotor.set(ControlMode.Velocity, speed);
  }
  public void stop(){
    shooterMotor.stopMotor();
  }
  
}
