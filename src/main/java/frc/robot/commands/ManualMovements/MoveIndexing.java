// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ManualMovements;

import edu.wpi.first.wpilibj.Encoder.IndexingType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexing;

public class MoveIndexing extends CommandBase {
  Indexing indexing;

  public MoveIndexing(Indexing e) {
    indexing = e;
    addRequirements(indexing);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    indexing.moveForward();
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      indexing.indexingStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
