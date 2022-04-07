// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ManualMovements.Turret;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShuffleBoardConfig;

public class LaunchBall extends CommandBase {
  Shooter shooty;
  Limelight lighty;
  private double distance;
  private double speed;
  private boolean finish;
  private boolean shuffle;
  private ShuffleBoardConfig config;
  public LaunchBall(Shooter shooty, Limelight lighty, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooty = shooty;
    this.speed = speed;
    this.lighty = lighty;
    distance = lighty.getDistanceToHoop();
    addRequirements(shooty, lighty);
  }
  public LaunchBall(Shooter shooty, Limelight lighty, double speed, boolean shuffle, ShuffleBoardConfig config) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooty = shooty;
    this.lighty = lighty;
    this.shuffle = shuffle;
    this.config = config;
    distance = lighty.getDistanceToHoop();
    this.speed = config.getRPMData(0);
    addRequirements(shooty, lighty);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finish = !finish;
    shooty.shootyLaunchyRun(speed);
  }

  // Called ev ery time the scheduler runs while the command is scheduled.
 public void execute() {
    shooty.shootyLaunchyRun(speed);
    //System.out.println("rpm = " + shooty.getShootyLaunchyVelocity() * 600 / 2048);
    System.out.println(distance);
    SmartDashboard.putNumber("Hoop distance", distance);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooty.shootyLaunchyRun(speed);
    //shooty.shootyLaunchyStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
