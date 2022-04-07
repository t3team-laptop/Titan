// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Toggles;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeMove;

public class ToggleIntake extends CommandBase {
  private IntakeMove intakeMove;
  private boolean down;
  private boolean auto;
  public ToggleIntake(IntakeMove in, boolean down, boolean auto) {
    this.intakeMove = in;
    this.down = down;
    this.auto = auto;
    addRequirements(intakeMove);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(down){
      intakeMove.move(Constants.INTAKE_MOVE_SPEED_DOWN);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(auto){
      intakeMove.move(Constants.INTAKE_MOVE_SPEED_DOWN);
    }
    else{
      intakeMove.intakeMoveStop();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public void toggleIntake(){
      down = !down;
  }
}
