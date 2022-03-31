// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  WPI_TalonFX elevatorPullR;
  WPI_TalonFX elevatorPullL;
  /** Creates a new Elevator. */
  public Elevator() {
    elevatorPullR = new WPI_TalonFX(Constants.ELEVATOR_MOTOR_PULL_R);
    elevatorPullL = new WPI_TalonFX(Constants.ELEVATOR_MOTOR_PULL_L);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //Move elevator
  public void elevatorPull(double speedR, double speedL){
    elevatorPullR.set(speedR);
    elevatorPullL.set(speedL);
  }

  public void elevatorPullStop(){
    elevatorPullR.stopMotor();
    elevatorPullL.stopMotor();
  }
}
