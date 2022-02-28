// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  //Initialize variables
  WPI_TalonSRX intakeMotor;
  WPI_TalonSRX intakeMoveMotor;

  public Intake() {
    intakeMotor = new WPI_TalonSRX(Constants.INTAKE_MOTOR);
    intakeMoveMotor = new WPI_TalonSRX(Constants.INTAKE_MOVE_MOTOR);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //Intake move methods
  public void move(int vel, double intakeMoveSpeed){
    intakeMoveMotor.set(intakeMoveSpeed * vel);
  }
  public void intakeMoveStop(){
    intakeMoveMotor.stopMotor();
  }

  //Intake run methods
  public void runIntake(int vel) {
    intakeMotor.set(Constants.INTAKE_SPEED * vel);
  }
  public void stopIntake() {
    intakeMotor.stopMotor();
  }
}
