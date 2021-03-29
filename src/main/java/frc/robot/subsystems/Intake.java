// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private WPI_TalonSRX intake;
  public Intake() {
    intake = new WPI_TalonSRX(Constants.INTAKE);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void intakeGo() {intake.set(Constants.INTAKE_SPEED);}
  public void intakeStop() {intake.set(0.0);}
}
