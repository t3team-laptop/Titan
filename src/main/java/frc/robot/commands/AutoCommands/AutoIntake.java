// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Indexing;
import frc.robot.subsystems.Intake;

public class AutoIntake extends CommandBase {
  Indexing indexing;
  Intake intake;
  Timer timer;
  private boolean finish = false;
  /** Creates a new AutoIntake. */
  public AutoIntake(Indexing indexing, Intake intake) {
    this.indexing = indexing;
    this.intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(indexing, intake);
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    intake.move(-1, Constants.INTAKE_MOVE_SPEED_DOWN);
    if(timer.get() == 1.0){
      intake.intakeMoveStop();
    }
    while(timer.get() > 1.5 && timer.get() < Constants.AUTO_INTAKE_TIME){      
      intake.runIntake(1);
      indexing.moveForward();
    }
    intake.stopIntake();
    indexing.indexingStop();
    finish = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.intakeMoveStop();
    intake.stopIntake();
    indexing.indexingStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
