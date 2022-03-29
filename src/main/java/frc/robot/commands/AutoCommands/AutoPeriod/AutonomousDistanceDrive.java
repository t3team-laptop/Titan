// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.AutoPeriod;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class AutonomousDistanceDrive extends CommandBase {
  private DriveTrain driveTrain;
  private double curSensorPos, leftSensorVal, rightSensorVal, targetSensorPos, targetPosition, error;
  private boolean finish;
  /** Creates a new AutonomousThreeBall. */
  public AutonomousDistanceDrive(DriveTrain driveTrain, double targetPosition) { // targetPosition should be in inches
    this.targetPosition = targetPosition;
    this.driveTrain = driveTrain;
    addRequirements(driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.leftFront.configFactoryDefault();
    driveTrain.rightFront.configFactoryDefault();
    driveTrain.leftFront.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 5000);
    driveTrain.rightFront.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 5000);
    driveTrain.leftFront.setSelectedSensorPosition(0, 0, 5000);
    driveTrain.rightFront.setSelectedSensorPosition(0, 0, 5000);

    targetSensorPos = targetPosition; // some math equaiton to convert inches to a sensor position (find through testing and pushing)
    finish = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    leftSensorVal = driveTrain.leftFront.getSelectedSensorPosition();
    rightSensorVal = driveTrain.rightFront.getSelectedSensorPosition();
    curSensorPos = (leftSensorVal + rightSensorVal)/2;
    if(curSensorPos < targetSensorPos + 10){
      error = targetSensorPos - curSensorPos;
      driveTrain.driveForward(error * Constants.AUTO_DISTANCE_KP + Constants.MIN_AUTO_DRIVE_SPEED);
    }
    else{
      finish = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
