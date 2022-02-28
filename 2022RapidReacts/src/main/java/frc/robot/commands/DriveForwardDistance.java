// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class DriveForwardDistance extends CommandBase {
  DriveTrain driveTrain;
  private boolean finish = false;
  private double kDriveTick2Feet = 1.0 / 4096 * 6 * Math.PI / 12;

  /** Creates a new DriveForwardDistance. */
  public DriveForwardDistance(DriveTrain dt){
    driveTrain = dt;
    addRequirements(driveTrain); 
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.leftBack.follow(driveTrain.leftFront);
    driveTrain.rightBack.follow(driveTrain.rightFront);

    driveTrain.leftFront.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    driveTrain.rightFront.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

    driveTrain.leftFront.setSensorPhase(false);
    driveTrain.rightFront.setSensorPhase(true);

    driveTrain.leftFront.setSelectedSensorPosition(0, 0, 10);
    driveTrain.rightFront.setSelectedSensorPosition(0, 0, 10);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftPosition = driveTrain.leftFront.getSelectedSensorPosition() * kDriveTick2Feet;
    double rightPosition = driveTrain.rightFront.getSelectedSensorPosition() * kDriveTick2Feet;
    double distance = (leftPosition + rightPosition) / 2;

    if (distance < Constants.AUTONOMOUS_TARGET_DISTANCE) {
      driveTrain.driveForward(Constants.AUTONOMOUS_SPEED);
    } else {
      driveTrain.driveForward(0);
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
