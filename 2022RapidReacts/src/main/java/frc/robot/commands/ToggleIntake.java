// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Encoder.IndexingType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

public class ToggleIntake extends CommandBase {
  Intake intake;
  boolean moveUp;
  Timer timer;
  RobotContainer rc;
  private boolean finish = false;

  public ToggleIntake(Intake e, boolean moveUp, RobotContainer robc) {
    this.intake = e;
    this.moveUp = moveUp;
    rc = robc;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*While this looks counterintuitive, it is timing the motors moving up
    and down so as to lock the motors into place.*/
      
    //If moving up
    if(moveUp) {
      //Run motors in one direction for sufficient time to lift motors, then for a short time run it backwards in order to "lock"
        if(timer.get() < Constants.INTAKE_MOVEUP_TIME1) {
          intake.move(1, Constants.INTAKE_MOVE_SPEED_UP);
        }
        else if(timer.get() < (Constants.INTAKE_MOVEUP_TIME2 + Constants.INTAKE_MOVEUP_TIME1)){
          intake.move(-1, Constants.INTAKE_MOVE_SPEED_DOWN);
        }
        else{
          intake.intakeMoveStop();
          rc.intakeStatusToggle(!moveUp);
          finish = true;
        }
      }
      //If moving down
      else {
        //Run motors in one direction for sufficient time to unlock motors, then for a longer time run it backwards in order to swing down
        if(timer.get() < Constants.INTAKE_MOVEDOWN_TIME1) {
          intake.move(-1, Constants.INTAKE_MOVE_SPEED_DOWN);
        }
        else if(timer.get() < (Constants.INTAKE_MOVEDOWN_TIME2 + Constants.INTAKE_MOVEDOWN_TIME1)){
          intake.move(1, Constants.INTAKE_MOVE_SPEED_UP);
        }
        else{
          intake.intakeMoveStop();
          rc.intakeStatusToggle(!moveUp);
          finish = true;
        }
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
    return finish;
  }
}
