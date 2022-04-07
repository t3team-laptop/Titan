// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.AutoPeriod;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class AutoToggleTracking extends CommandBase {
  /** Creates a new AutoToggleTracking. */
  private Turret turret;
  public AutoToggleTracking(Turret turr) {
    // Use addRequirements() here to declare subsystem dependencies.
    turret = turr;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turret.toggleTracking();
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
    return true;
  }
}
