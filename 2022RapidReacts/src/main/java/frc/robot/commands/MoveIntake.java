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
  double moveSpeed;

  public MoveIntake(Intake e, int vel, double intakeMoveSpeedUp) {
    this.intake = e;
    moveVelocity = vel;
    moveSpeed = intakeMoveSpeedUp;
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
        intake.move(moveVelocity, moveSpeed);
      }
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.intakeMoveStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
