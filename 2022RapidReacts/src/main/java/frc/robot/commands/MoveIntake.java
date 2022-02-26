// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Encoder.IndexingType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class MoveIntake extends CommandBase {
  Intake intake;
  int moveVelocity;

  public MoveIntake(Intake e, int vel) {
    this.intake = e;
    moveVelocity = vel;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      if(moveVelocity == 0) {
        intake.intakeMoveStop(); //This may be unnecessary, check in later
      }
      else {
        intake.intakeMove(moveVelocity);
      }
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intaqke.intakeMoveStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
