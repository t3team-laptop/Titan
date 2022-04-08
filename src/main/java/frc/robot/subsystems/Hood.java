// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Hood extends SubsystemBase {
  //public CANSparkMax shooterHood;
  //public RelativeEncoder hoodEncoder;
  //public PIDController hoodPos;
  private double position;
  private double setpoint;
  private ArmFeedforward feedFor;
  /** Creates a new Hood. */
  public Hood() {
    //shooterHood = new CANSparkMax(Constants.SHOOTER_HOOD_PITCH, MotorType.kBrushless);
    //hoodEncoder = shooterHood.getEncoder();
    //hoodPos = new PIDController(0, 0, 0);
    //hoodPos.setTolerance(0.05);
    //feedFor = new ArmFeedforward(0, 0, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //position = getHoodPosition();
    //shooterHood.set(MathUtil.clamp(hoodPos.calculate(position, setpoint)+ feedFor.calculate(setpoint, 0), 0, 0.2));
    //SmartDashboard.putNumber("HoodPos", position);
  }

  // public RelativeEncoder getHoodEncoder(){
  //   //return hoodEncoder;
  // }

  // public PIDController getHoodPidController(){
  //   return hoodPos;
  // }
  // public void shooterHoodRun(double speed){
  //   shooterHood.set(speed);
  // }
  // public void shooterHoodStop(){
  //   shooterHood.stopMotor();
  // }
  // public double getHoodPosition(){
  //   return hoodEncoder.getPosition();
  // }
  public void updateSetpoint(double set){
    setpoint = set;
  }
}
