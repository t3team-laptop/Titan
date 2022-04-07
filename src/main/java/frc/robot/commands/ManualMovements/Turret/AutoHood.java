// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ManualMovements.Turret;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class AutoHood extends CommandBase {
  private Hood hoody;
  private double targetPosition, currentPosition, error;
  private Limelight limy;
  private double distance;
  private double hoodykP, hoodykI, hoodykD, hoodykIz, hoodykFF, hoodykMaxOutput, hoodykMinOutput;
  private RelativeEncoder hoodEncoder;
  private PIDController hoodPidController;
  private boolean finish;
  private boolean pos1;
  private boolean pos2;
  private boolean pos3;
  /** Creates a new AdjustHood. */
  public AutoHood(Hood hooy, Limelight limy, int pos) {
    this.hoody = hooy;
    this.limy = limy;
    addRequirements(hoody, limy);
    distance = limy.getDistanceToHoop();
    hoodEncoder = hoody.getHoodEncoder();
    hoodEncoder.setPosition(0);
    hoodPidController = hoody.getHoodPidController();
    if(pos == 1){
      pos1 = true;
      pos2 = false;
      pos3 = false;
    }else if(pos == 2){
      pos1 = false;
      pos2 = true;
      pos3 = false;
    }else if(pos == 3){
      pos1 = false;
      pos2 = false;
      pos3 = true;
    }else{
      pos1 = false;
      pos2 = false;
      pos3 = false;
    }
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // currentPosition = hoodEncoder.getPosition();
    // if(currentPosition < targetPosition + marginOfError){
    //   error = targetPosition - currentPosition;
    //   shooty.shooterHoodRun(error * Constants.AUTO_TURNING_KP + 0.1);
    // }
    // else if(currentPosition > targetPosition - marginOfError){
    //   error = targetPosition - currentPosition;
    //   shooty.shooterHoodRun(error * Constants.AUTO_TURNING_KP*-1 - 0.1);
    // }
    // else{
    //   finish = true;
    // }
    if(pos1){
      hoody.updateSetpoint(0.15);
    }else if(pos2){
      hoody.updateSetpoint(0.25);
    }else if(pos3){
      hoody.updateSetpoint(0.45);
    }else{
      hoody.updateSetpoint(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }

  //Was in execute
  //Equation to find targetPosition of hood based off distance - Add this here
    // while((hoodEncoder.getPosition() > targetPosition + marginOfError) || (hoodEncoder.getPosition() < targetPosition - marginOfError)){
    //   hoodyI = hoodEncoder.getPosition() - targetPosition;
    //   shooty.shooterHoodRun(hoodyKp * hoodyI);
    // }
    // shooty.shooterHoodRun(Constants.SHOOTER_HOOD_UP_SPEED);
}
