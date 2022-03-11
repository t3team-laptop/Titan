// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.Math;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight extends SubsystemBase {
  public WPI_TalonSRX turretFinderMotor;

  private NetworkTable table;
  private NetworkTableEntry tx;
  private NetworkTableEntry ty;
  private NetworkTableEntry ta;

  private double x; // Horizontal offset of crosshair from target
  private double y; // Vertical offset of crosshair from target
  private double area; // Amount of area target takes up on camera (0-100%)

  private double distanceToHoop;

  private double threshold;
  private double Kp;
  private double targetAngleX;
  private double targetAngleY;
  private double prevEroor;
  private double turnSpeed;
  private double integralX;
  /** Creates a new Limelight. */
  public Limelight() {
    turretFinderMotor = new WPI_TalonSRX(Constants.TURRET_FINDER_MOTOR);
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    Kp = Constants.KP;
    //figure out what we want for it during tuning for anything below this line
    threshold = 3;
    targetAngleX = 0;
    targetAngleY = 0;
    prevEroor = 0;
    turnSpeed = 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    distanceToHoop = getDistanceToHoop();
  }

  // public boolean calclateRotateValue(double targetAngle){
  //   double error = targetAngle - Constants.LIMELIGHT_MOUNTING_ANGLE_DEGREES;
  //   if(error > threshold){
  //     rotation = error*Kp;
  //     return false;
  //   }
  //   else{
  //     rotation = 0;
  //     return true;
  //   }
  // }

  public double PID(double target){
    double error = target - Constants.LIMELIGHT_MOUNTING_ANGLE_DEGREES;
    this.integralX += (error*.02);
    double derivative = (error - this.prevEroor)/.2;
    return Constants.TURRETXP*error + Constants.TURRETXI*this.integralX + Constants.TURRETXD*derivative;
  }

  public double getDistanceToHoop(){
      //read values periodically
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);

    // how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = Constants.LIMELIGHT_MOUNTING_ANGLE_DEGREES;

    // distance from the center of the Limelight lens to the floor
    double limelightLensHeight = Constants.LIMELIGHT_LENS_HEIGHT; // inches

    // distance from the target to the floor
    double hoopHeight = 104.0;

    double angleToGoalDegrees = limelightMountAngleDegrees + y;
    double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);

    //calculate distance
    distanceToHoop = (hoopHeight - limelightLensHeight)/Math.tan(angleToGoalRadians);

    //post to smart dashboard periodically    
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);

    return distanceToHoop;
  }

  public void runTurretFinder(double vel) {
    turretFinderMotor.set(vel);
  }

  public double getX(){
    return x;
  }

  public boolean hasTarget(){
    return y == 0.0;
  }

  public void stopTurretFinder() {
    turretFinderMotor.stopMotor();
  }
}