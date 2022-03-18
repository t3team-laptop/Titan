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

<<<<<<< HEAD
  //Rosbots Code
  /**
   *
   * @param shooter shooter subsystem
   * @param rps     Hardcoded setpoint for Shooter
   */
  public LaunchBall(Shooter shooter, double rps) {
    this.shooty = shooter;
    this.setRPS = rps;
    addRequirements(shooty);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Rosbots Code
    if (this.lighty == null && this.setRPS > 0) {
      this.shooty.setSetpoint(this.setRPS);
    } else {
      updateSetpoint();
    }
    this.shooty.enableShooter();
  }

  // Called every time the scheduler runs while the command is scheduled.
  public void execute() {
    // fullSpeed = true;
    // shooty.shootyLaunchyRun(Constants.SHOOTER_LAUNCH_SPEED);
    // SmartDashboard.putBoolean("Full speed", fullSpeed);

    //Rosbots Code
    if (this.lighty != null) {
      updateSetpoint();
    }
=======
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
 public void execute() {
    fullSpeed = true;
    shooty.shootyLaunchyRun(speed);
    SmartDashboard.putBoolean("Full speed", fullSpeed);
>>>>>>> 03c27405fcc6da66c7639b1daaf02e003ca12cc1
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooty.shootyLaunchyRun(Constants.SHOOTER_LAUNCH_IDLE_SPEED);
    fullSpeed = false;
    SmartDashboard.putBoolean("Full Speed", fullSpeed);
  }

<<<<<<< HEAD
  private void updateSetpoint() {
    double distance = lighty.getDistance() / 12;
    // System.out.println("Vision Distance: " + distance);
    newDisRPM = 4.10774 * Math.pow(distance, 3) - 126.12794 * Math.pow(distance, 2)
        + 1368.53535 * Math.pow(distance, 1) - 1095 - 100;

    // newDisRPM =
    // 3.452380952381 * Math.pow(distance, 3) - 61.7857142857143 * Math.pow(distance, 2)
    // + 402.6190476190476 * Math.pow(distance, 1) + 3100;

    // 5.5ft - 3300rpm
    // 9ft - 4000rpm
    // 13.5ft - 4500rpm
    // 16.5ft - 5600rpm



    // newDisRPM = 3.45238 * Math.pow(distance, 3) - 51.42857 * Math.pow(distance, 2)
    // + 289.40476 * Math.pow(distance, 1) + 3300;
    // newDisRPM = 3.45238 * Math.pow(distance, 3) - 56 * Math.pow(distance, 2)
    // + 345 * Math.pow(distance, 1) + 2975;
    if (Math.abs(curDisRPM - newDisRPM) >= 100) {
        if (newDisRPM >= 6000) {
            curDisRPM = 6000;
        } else if (newDisRPM <= 3500) {
            curDisRPM = 3500;
        } else {
            curDisRPM = newDisRPM;
        }
        this.shooty.setSetpoint(curDisRPM / 60);
    }
}

=======
>>>>>>> 03c27405fcc6da66c7639b1daaf02e003ca12cc1
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
