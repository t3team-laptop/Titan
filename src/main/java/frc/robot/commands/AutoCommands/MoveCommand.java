// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class MoveCommand extends CommandBase {
  private DriveTrain driveTrain;
  private double distance, rotation;
  private double leftSensorVal, rightSensorVal, avgSensorVal, currentDistance;
  private boolean forwy;
  /** Creates a new Path. */
  public MoveCommand(DriveTrain dt, double dis, double rot, boolean forw){
    // Use addRequirements() here to declare subsystem dependencies.
    driveTrain = dt;
    distance = dis;
    rotation = rot;
    forwy = forw;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.leftFront.configFactoryDefault();
    driveTrain.rightFront.configFactoryDefault();

    driveTrain.leftFront.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 500);
    driveTrain.rightFront.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 500);

    //driveTrain.leftFront.setSensorPhase(false);
    //driveTrain.rightFront.setSensorPhase(true);

    driveTrain.leftFront.setSelectedSensorPosition(0, 0, 500);
    driveTrain.rightFront.setSelectedSensorPosition(0, 0, 500);

    driveTrain.curvyMove(rotation, forwy);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    leftSensorVal = driveTrain.leftFront.getSelectedSensorPosition();
    rightSensorVal = driveTrain.rightFront.getSelectedSensorPosition();
    avgSensorVal = (leftSensorVal + rightSensorVal)/2;
    currentDistance = nativeUnitsToDistanceFeet(avgSensorVal);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return currentDistance >= distance;
  }

  private double nativeUnitsToDistanceFeet(double sensorCounts){
    double motorRotations = (double)sensorCounts / Constants.FALCON_COUNTSPERREV;
    double wheelRotations = motorRotations / Constants.DRIVE_GEARRATIO;
    double positionFeet = wheelRotations * (2 * Math.PI * (Constants.DRIVE_WHEELRADIUS/12));
    return positionFeet;
  }
}
