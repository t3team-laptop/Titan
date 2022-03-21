// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  WPI_TalonSRX elevatorPullR;
  WPI_TalonSRX elevatorPullL;
  WPI_TalonSRX elevatorTiltR;
  WPI_TalonSRX elevatorTiltL;
  /** Creates a new Elevator. */
  public Elevator() {
    elevatorPullR = new WPI_TalonSRX(Constants.ELEVATOR_MOTOR_PULL_R);
    elevatorPullL = new WPI_TalonSRX(Constants.ELEVATOR_MOTOR_PULL_L);
    elevatorTiltR = new WPI_TalonSRX(Constants.ELEVATOR_TILT_R);
    elevatorTiltL = new WPI_TalonSRX(Constants.ELEVATOR_TILT_L);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void elevatorPull(double speed){
    elevatorPullR.set(speed);
    elevatorPullL.set(speed);
  }

  public void elevatorPullStop(){
    elevatorPullR.stopMotor();
    elevatorPullL.stopMotor();
  }

  public void elevatorTiltBack(double speed){
    elevatorTiltR.set(speed);
    elevatorTiltL.set(speed);
  }

  public void elevatorTiltStop(){
    elevatorTiltR.stopMotor();
    elevatorTiltL.stopMotor();
  }

  public WPI_TalonSRX getElevatorTiltMotor(){
    return elevatorTiltR;
  }
}