// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class CenterTarget extends CommandBase {
  private Turret turret;
  private Limelight limelight;
  private Shooter shooty;
  /** Creates a new CenterTarget. */
  public CenterTarget(Turret turret, Limelight limelight, Shooter shoot) {
    this.turret = turret;
    this.limelight = limelight;
    this.shooty = shoot;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(this.turret.getTrackingOn() && this.limelight.hasTarget()){
      double val = limelight.getX();
      this.turret.runTurretFinder(val);
      SmartDashboard.putNumber("HUB Dis", limelight.getDistanceToHoop());
      //updateRPM();
    }else{
      this.turret.runTurretFinder(0);
    }
    //this.limelight.setLEDMode(this.turret.getTrackingOn());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public void updateRPM(){
    double x = limelight.getDistanceToHoop();
    shooty.shootyLaunchyRun(MathUtil.clamp(3*x*x + 2*x + 4, Constants.SHOOTER_MINSPEED, Constants.SHOOTER_MAXSPEED));
  }
}
