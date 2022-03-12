// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class ManualHood extends CommandBase {
  private Shooter shooty;
  private boolean up;
  /** Creates a new ManualHood. */
  public ManualHood(Shooter shooty, boolean up) {
    this.shooty = new Shooter();
    this.up = up;
    addRequirements(shooty);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(up){
      shooty.shooterHoodRun(Constants.MANUAL_SHOOTER_HOOD);
    }
    else if(!up){
      shooty.shooterHoodRun(Constants.MANUAL_SHOOTER_HOOD * -1);
    }
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
