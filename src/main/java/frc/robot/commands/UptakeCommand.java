// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Uptake;

public class UptakeCommand extends CommandBase {
  /** Creates a new UptakeRun. */
  private final Uptake uptake;
  private final boolean reverse;
  public UptakeCommand(Uptake u, boolean r) {
    // Use addRequirements() here to declare subsystem dependencies.
    uptake = u;
    reverse = r;
    addRequirements(uptake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    uptake.uptakeGo(reverse);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {uptake.uptakeStop();}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
