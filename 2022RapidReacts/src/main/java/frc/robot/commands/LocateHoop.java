// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants;

public class LocateHoop extends CommandBase {
  Shooter shoot; 
  public double turretSpeed;
  public double minTurretSpeed;
  public double Kp;
  public double heading_error;
  /** Creates a new ShootBall. */
  public LocateHoop(Shooter shoot) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shoot = shoot;
    addRequirements(shoot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turretSpeed = Constants.TURRET_ADJUST_SPEED;
    minTurretSpeed = Constants.MINIMUM_TURRET_ADJUST_SPEED;
    Kp = Constants.KP;
    heading_error = -shoot.x;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shoot.y == 0.0)
    {
        // We don't see the target, seek for the target by spinning in place at a safe speed.
        //turretSpeed = 0.5;
    }
    else
    {
      //Check that we do need the 1.0 for each side
      // We do see the target, execute aiming code
      if (shoot.x > 1.0)
      {
              turretSpeed = Kp*heading_error - minTurretSpeed;
      }
      else if (shoot.x < 1.0)
      {
              turretSpeed = Kp*heading_error + minTurretSpeed;
      }
    }
    shoot.runTurretFinder(turretSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shoot.stopTurretFinder();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
