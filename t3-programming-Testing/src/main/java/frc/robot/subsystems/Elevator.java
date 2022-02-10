// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  //Initialize variables
  WPI_TalonSRX elevatorMotor;


  public Elevator() {
    elevatorMotor = new WPI_TalonSRX(Constants.ELEVATOR_MOTOR);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void moveUp(){
    elevatorMotor.set(Constants.ELEVATOR_SPEED);
  }
  public void moveDown(){
    elevatorMotor.set(-Constants.ELEVATOR_SPEED);
  }
  public void elevatorStop(){
    elevatorMotor.stopMotor();
  }
}
