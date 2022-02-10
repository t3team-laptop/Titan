// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class MoveElevator extends CommandBase {
  Elevator elevator;
  boolean up;

  public MoveElevator(Elevator e, boolean u) {
    elevator = e;
    up = u;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(up){
      elevator.moveUp();
    }else{
      elevator.moveDown();
    }
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {elevator.elevatorStop();}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
