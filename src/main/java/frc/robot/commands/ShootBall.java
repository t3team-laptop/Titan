// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallSuck;
import frc.robot.commands.ShooterState;

public class ShootBall extends CommandBase {
  /** Creates a new ShootBall. */
  private final BallSuck ballSuck;
  public ShootBall(BallSuck bs) {
    ballSuck = bs;
    addRequirements(ballSuck);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (ShooterState.getState()){ballSuck.ballSuckGo();}
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {ballSuck.ballSuckStop();}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
