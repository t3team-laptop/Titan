// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ManualMovements.Turret;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class ManualHood extends CommandBase {
  private Hood hoody;
  private boolean up;
  private double distance;
  private RelativeEncoder hoodEncoder;
  /** Creates a new ManualHood. */
  public ManualHood(Hood hooy, boolean up, Limelight limelight) {
    this.hoody = hooy;
    this.up = up;
    distance = limelight.getDistanceToHoop();
    //hoodEncoder = hoody.getHoodEncoder();
    addRequirements(hoody);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(up){
      //shooty.shooterHoodRun(Constants.MANUAL_SHOOTER_HOOD_UP);
      //hoody.shooterHoodRun(0.075);
    }
    else if(!up){
      //shooty.shooterHoodRun(Constants.MANUAL_SHOOTER_HOOD_DOWN * -1);
      //hoody.shooterHoodRun(-0.025);
    }
    //position of the encoder in units of revolutions
    SmartDashboard.putNumber("Hood Encoder Position", hoodEncoder.getPosition());

    ////System.out.println("hood position "+ hoodEncoder.getPosition());
    ////System.out.println("distance " + distance);

    SmartDashboard.putNumber("Distance to target", distance);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //hoody.shooterHoodStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
