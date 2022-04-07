// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hood extends SubsystemBase {
  public CANSparkMax shooterHood;
  public RelativeEncoder hoodEncoder;
  public PIDController hoodPIDController;
  /** Creates a new Hood. */
  public Hood() {
    shooterHood = new CANSparkMax(Constants.SHOOTER_HOOD_PITCH, MotorType.kBrushless);
    hoodEncoder = shooterHood.getEncoder();
    hoodPIDController = new PIDController(0, 0, 0);
    hoodPIDController.setTolerance(0.05);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public RelativeEncoder getHoodEncoder(){
    return hoodEncoder;
  }

  public PIDController getHoodPidController(){
    return hoodPIDController;
  }
  public void shooterHoodRun(double speed){
    shooterHood.set(speed);
  }
  public void shooterHoodStop(){
    shooterHood.stopMotor();
  }
  public double getHoodPosition(){
    return hoodEncoder.getPosition();
  }
}
