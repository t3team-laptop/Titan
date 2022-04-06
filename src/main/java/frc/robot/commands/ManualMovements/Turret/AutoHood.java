// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ManualMovements.Turret;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

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
  private SparkMaxPIDController hoodPidController;
  private boolean finish;
  /** Creates a new AdjustHood. */
  public AutoHood(Hood hooy, Limelight limy, double targetPosition) {
    this.hoody = hooy;
    this.limy = limy;
    this.targetPosition = targetPosition;
    addRequirements(hoody, limy);
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hoodEncoder.setPosition(0);
    distance = limy.getDistanceToHoop();
    hoodEncoder = hoody.getHoodEncoder();
    hoodPidController = hoody.getHoodPidController();
    hoodykP = 0;
    hoodykI = 0;
    hoodykD = 0;
    hoodykIz = 0;
    hoodykFF = 0;
    hoodykMaxOutput = 1;
    hoodykMinOutput = -1;
    hoodPidController.setP(hoodykP);
    hoodPidController.setI(hoodykI);
    hoodPidController.setD(hoodykD);
    hoodPidController.setIZone(hoodykIz);
    hoodPidController.setFF(hoodykFF);
    hoodPidController.setOutputRange(hoodykMinOutput, hoodykMaxOutput);
    
    
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
    hoodPidController.setReference(targetPosition, CANSparkMax.ControlType.kVoltage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hoody.shooterHoodStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }

  //Was in execute
  //Equation to find targetPosition of hood based off distance - Add this here
    // while((hoodEncoder.getPosition() > targetPosition + marginOfError) || (hoodEncoder.getPosition() < targetPosition - marginOfError)){
    //   hoodyI = hoodEncoder.getPosition() - targetPosition;
    //   shooty.shooterHoodRun(hoodyKp * hoodyI);
    // }
    // shooty.shooterHoodRun(Constants.SHOOTER_HOOD_UP_SPEED);
}
