// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class LaunchBall extends CommandBase {
  Shooter shooty;
  Limelight lighty;
  private double distance;
  private boolean fullSpeed;
  public LaunchBall(Shooter shooty, Limelight lighty) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooty = shooty;
    this.lighty = lighty;
    distance = lighty.getDistanceToHoop();
    addRequirements(shooty, lighty);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooty.shootyLaunchyRun(Constants.SHOOTER_LAUNCH_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
 public void execute() {
    fullSpeed = true;
    shooty.shootyLaunchyRun(Constants.SHOOTER_LAUNCH_SPEED);
    SmartDashboard.putBoolean("Full speed", fullSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooty.shootyLaunchyRun(Constants.SHOOTER_LAUNCH_SPEED);
    fullSpeed = false;
    SmartDashboard.putBoolean("Full Speed", fullSpeed);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
