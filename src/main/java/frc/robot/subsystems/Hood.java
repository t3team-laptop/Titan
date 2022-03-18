// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

public class Hood extends PIDSubsystem {
  public BuiltInWidgets kPIDCommand;
  private ShuffleboardTab hoodTab;
  private CANSparkMax hoodMotor;
  private Limelight limy;
  private double position;
  private double targetPosition;
  private RelativeEncoder encoder;
  private NetworkTableEntry shuffleboardShit;
  /** Creates a new Hood. */
  public Hood(Limelight limy) {
    super(new PIDController(Constants.HOOD_PITCH_KP, Constants.HOOD_PITCH_KI, Constants.HOOD_PITCH_KD));
    hoodTab = Shuffleboard.getTab("Hood PID");
    hoodMotor = new CANSparkMax(Constants.SHOOTER_HOOD_PITCH, MotorType.kBrushless);
    encoder = hoodMotor.getEncoder();
    this.limy = limy;
    hoodMotor.getEncoder();
    shuffleboardShit = Shuffleboard.getTab("Hood PID").addPersistent("KP", Constants.HOOD_PITCH_KP).withWidget(BuiltInWidgets.kPIDCommand).getEntry();
    shuffleboardShit = Shuffleboard.getTab("Hood PID").addPersistent("KI", Constants.HOOD_PITCH_KI).withWidget(BuiltInWidgets.kPIDCommand).getEntry();
    shuffleboardShit = Shuffleboard.getTab("Hood PID").addPersistent("KD", Constants.HOOD_PITCH_KD).withWidget(BuiltInWidgets.kPIDCommand).getEntry();
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return 0;
  }
  public void setHoodPosition(){
    
  }
  public RelativeEncoder getHoodEncoder(){
    return encoder;
  }
  public void shooterHoodRun(double speed){
    hoodMotor.set(speed);
  }
  public void shooterHoodStop(){
    hoodMotor.stopMotor();
  }

  public void enableHood(){
    this.enable();
  }
}
