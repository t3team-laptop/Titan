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
  private double distance, twisty;
  private double leftSensorVal, rightSensorVal, avgSensorVal, currentDistance;
  private boolean forwy, quicky;
  /** Creates a new Path. */
  public MoveCommand(DriveTrain dt, double dis, double rot, boolean forw, boolean quick){
    // Use addRequirements() here to declare subsystem dependencies.
    leftSensorVal = 0;
    rightSensorVal = 0;
    avgSensorVal = 0;
    currentDistance = 0;
    driveTrain = dt;
    distance = dis;
    twisty = rot;
    forwy = forw;
    quicky = quick;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.leftFront.configFactoryDefault();
    driveTrain.rightFront.configFactoryDefault();

    driveTrain.leftFront.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 5000);
    driveTrain.rightFront.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 5000);

    //driveTrain.leftFront.setSensorPhase(false);
    //driveTrain.rightFront.setSensorPhase(true);

    driveTrain.leftFront.setSelectedSensorPosition(0, 0, 5000);
    driveTrain.rightFront.setSelectedSensorPosition(0, 0, 5000);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    leftSensorVal = driveTrain.leftFront.getSelectedSensorPosition();
    rightSensorVal = driveTrain.rightFront.getSelectedSensorPosition();
    System.out.println(leftSensorVal);
    avgSensorVal = (leftSensorVal + rightSensorVal)/2;
    currentDistance = nativeUnitsToDistanceFeet(avgSensorVal);
    driveTrain.driveForward(-0.7);
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
    double motortwistys = (double) sensorCounts / Constants.FALCON_COUNTSPERREV;
    double wheeltwistys = motortwistys / Constants.DRIVE_GEARRATIO;
    double positionFeet = wheeltwistys * (2 * Math.PI * (Constants.DRIVE_WHEELRADIUS/12));
    return positionFeet;
  }
}
