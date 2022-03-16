// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class ElevatorPull extends CommandBase {
  private Elevator elevator;
  private boolean positive;
  /** Creates a new ElevatorPull. */
  public ElevatorPull(Elevator elevator, boolean positive) {
    this.elevator = elevator;
    this.positive = positive;
    addRequirements(elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(positive){
      elevator.elevatorPull(Constants.ELEVATOR_PULL_SPEED);
    }
    else if(!positive){
      elevator.elevatorPull(Constants.ELEVATOR_PULL_SPEED * -1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.elevatorPullStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
