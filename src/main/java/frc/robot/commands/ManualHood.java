// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class ManualHood extends CommandBase {
  private Shooter shooty;
  private Limelight limelight;
  private boolean up;
  private double distance;
  private RelativeEncoder hoodEncoder;
  /** Creates a new ManualHood. */
  public ManualHood(Shooter shooty, boolean up, Limelight limelight) {
    this.shooty = shooty;
    this.up = up;
    this.limelight = limelight;
    distance = limelight.getDistanceToHoop();
    hoodEncoder = shooty.getHoodEncoder();
    addRequirements(shooty);
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
      shooty.shooterHoodRun(Constants.MANUAL_SHOOTER_HOOD_UP);
    }
    else if(!up){
      shooty.shooterHoodRun(Constants.MANUAL_SHOOTER_HOOD_DOWN * -1);
    }
    //position of the encoder in units of revolutions
    SmartDashboard.putNumber("Hood Encoder Position", hoodEncoder.getPosition());

    SmartDashboard.putNumber("Distance to target", distance);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooty.shooterHoodStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
