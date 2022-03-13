// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeMove;

public class ToggleIntake extends CommandBase {
  IntakeMove intakeMove;
  boolean moveUp;

  public ToggleIntake(IntakeMove in) {
    moveUp = false;
    this.intakeMove = in;
    addRequirements(intakeMove);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    moveLoop();
    moveUp = !moveUp;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  //Movement loop
  public void moveLoop() {
    //Depending on down/up status, hold the motor in one of two positions.
    while(true) {
      if(moveUp) {
        intakeMove.move(1, 0.4);
      }
      else {
        intakeMove.intakeMoveStop();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeMove.intakeMoveStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
