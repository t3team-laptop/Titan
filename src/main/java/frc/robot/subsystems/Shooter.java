// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  WPI_TalonSRX shooterMotor;
  /** Creates a new Shooter. */
  public Shooter() {
    shooterMotor = new WPI_TalonSRX(Constants.SHOOTER_MOTOR);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void shooterRevUP(double speed){
    shooterMotor.set(speed);
  }
  public void stop(){
    shooterMotor.stopMotor();
  }
  
}
