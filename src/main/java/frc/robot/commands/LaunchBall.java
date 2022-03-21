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
  private double speed;
  private boolean fullSpeed;
  public LaunchBall(Shooter shooty, Limelight lighty, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooty = shooty;
    this.speed = speed;
    this.lighty = lighty;
    distance = lighty.getDistanceToHoop();
    addRequirements(shooty, lighty);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooty.shootyLaunchyRun(speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
 public void execute() {
    fullSpeed = true;
    shooty.shootyLaunchyRun(speed);
    System.out.println("rpm = " + shooty.getShootyLaunchyVelocity() * 600 / 2048);
    SmartDashboard.putBoolean("Full speed", fullSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooty.shootyLaunchyRun(speed);
    fullSpeed = false;
    SmartDashboard.putBoolean("Full Speed", fullSpeed);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
