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

  public NetworkTable table;
  public NetworkTableEntry tx;
  public NetworkTableEntry ty;
  public NetworkTableEntry ta;

  public double x; // Horizontal offset of crosshair from target
  public double y; // Vertical offset of crosshair from target
  public double area; // Amount of area target takes up on camera (0-100%)

  public double distanceToHoop;
  
  /** Creates a new Limelight. */
  public Limelight() {
    turretFinderMotor = new WPI_TalonSRX(Constants.TURRET_FINDER_MOTOR);
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    getDistanceToHoop();
  }

  public void getDistanceToHoop(){
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
  }

  public void runTurretFinder(double vel) {
    turretFinderMotor.set(vel);
  }

  public void stopTurretFinder() {
    turretFinderMotor.stopMotor();
  }
}