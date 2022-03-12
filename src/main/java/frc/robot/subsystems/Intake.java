// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  //Initialize variables
  private WPI_TalonSRX intakeMotor;
  private CANSparkMax intakeMoveMotor;

  public Intake() {
    intakeMotor = new WPI_TalonSRX(Constants.INTAKE_MOTOR);
    intakeMoveMotor = new CANSparkMax(Constants.INTAKE_MOVE_MOTOR, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //Intake move methods
  // //public void move(int vel, double intakeMoveSpeed){
  //   System.out.println("Trying to move with vel: " + vel + ", and moveSpeed: " + intakeMoveSpeed);
  //   intakeMoveMotor.set(intakeMoveSpeed * vel);
  // }
  public void intakeMoveStop(){
    //intakeMoveMotor.stopMotor();
    System.out.println("Trying to stop intake moveMotor.");
  }

  //Intake run methods
  public void runIntake() {
    intakeMotor.set(Constants.INTAKE_SPEED);
  }
  public void stopIntake() {
    intakeMotor.stopMotor();
  }
}