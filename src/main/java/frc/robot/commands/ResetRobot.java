// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;

public class ResetRobot extends CommandBase {
  Limelight limy;
  public ResetRobot(Limelight limy) {
    this.limy = limy;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limy.turretFinderMotor.configFactoryDefault();
    limy.turretFinderMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 5000);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
