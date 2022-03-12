// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class ToggleIntake extends CommandBase {
  Intake intake;
  boolean moveUp;
  Timer timer;
  boolean finished;

  public ToggleIntake(Intake in) {
    moveUp = false;
    finished = false;
    timer = new Timer();
    this.intake = in;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    finished = false;
    moveLoop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  //Movement loop
  public void moveLoop() {
    /*While this looks counterintuitive, it is timing the motors moving up
    and down so as to lock the motors into place.*/
    while(!finished) {      
      //If moving up
      if(moveUp) {
        //Run motors in one direction for sufficient time to lift motors, then for a short time run it backwards in order to "lock"
          if(timer.get() < Constants.INTAKE_MOVEUP_TIME1) {
            intake.move(1, Constants.INTAKE_MOVE_SPEED_UP);
            //System.out.println("Moving Up");
          }
          else if(timer.get() < (Constants.INTAKE_MOVEUP_TIME2 + Constants.INTAKE_MOVEUP_TIME1)){
            intake.move(-1, Constants.INTAKE_MOVE_SPEED_DOWN);
            //System.out.println("Moving down to lock up");
          }
          else{
            intake.intakeMoveStop();
            moveUp = !moveUp;
            timer.reset();
            finished = true;
            break;
          }
        }
        //If moving down
        else {
          //Run motors in one direction for sufficient time to unlock motors, then for a longer time run it backwards in order to swing down
          if(timer.get() < Constants.INTAKE_MOVEDOWN_TIME1) {
            intake.move(-1, Constants.INTAKE_MOVE_SPEED_DOWN);
            //System.out.println("Moving Down");
          }
          else if(timer.get() < (Constants.INTAKE_MOVEDOWN_TIME2 + Constants.INTAKE_MOVEDOWN_TIME1)){
            intake.move(1, Constants.INTAKE_MOVE_SPEED_UP);
            //System.out.println("Moving up to lock down");
          }
          else{
            intake.stopIntake();
            moveUp = !moveUp;
            timer.reset();
            finished = true;
            System.out.println("Finished moving.");
            break;
          }
        }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //intake.intakeMoveStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
