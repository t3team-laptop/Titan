// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Indexing extends SubsystemBase {
  //Initialize variables
  WPI_TalonSRX[] indexingMotors = new WPI_TalonSRX[2];


  public Indexing() {
    indexingMotors[0] = new WPI_TalonSRX(Constants.INDEX_LEFT);
    indexingMotors[1] = new WPI_TalonSRX(Constants.INDEX_RIGHT);
    indexingMotors[0].setNeutralMode(NeutralMode.Brake);
    indexingMotors[1].setNeutralMode(NeutralMode.Brake);
    indexingMotors[1].setInverted(true);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //Run and stop the indexing motors
  public void moveForward(){
    indexingMotors[0].set(Constants.INDEX_SPEED);
    indexingMotors[1].set(Constants.INDEX_SPEED);
  }
  public void indexingStop(){
    indexingMotors[0].stopMotor();
    indexingMotors[1].stopMotor();
  }
}
