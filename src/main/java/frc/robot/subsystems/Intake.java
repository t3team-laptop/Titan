// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  //Initialize variables
  private WPI_TalonSRX intakeMotor;

  public Intake() {
    intakeMotor = new WPI_TalonSRX(Constants.INTAKE_MOTOR);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //Intake run methods
  public void runIntake() {
    intakeMotor.set(Constants.INTAKE_SPEED);
  }
  public void runIntakeBack(){
    intakeMotor.set(Constants.INTAKE_SPEED * -1);
  }
  public void stopIntake() {
    intakeMotor.stopMotor();
  }
}