// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeMove extends SubsystemBase {
  //Initialize variables
  private CANSparkMax intakeMoveMotor;

  public IntakeMove() {
    intakeMoveMotor = new CANSparkMax(Constants.INTAKE_MOVE_MOTOR, MotorType.kBrushless);
    intakeMoveMotor.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //runs and stops the intake move motor
  public void move(double intakeMoveSpeed){
    intakeMoveMotor.set(intakeMoveSpeed);
  }
  public void intakeMoveStop(){
    intakeMoveMotor.stopMotor();
    //System.out.println("Trying to stop intake moveMotor.");
  }
}


