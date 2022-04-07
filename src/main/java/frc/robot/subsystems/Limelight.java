// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.Math;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight extends SubsystemBase {
  //public WPI_TalonFX turretFinderMotor;
  private double disX;
  private NetworkTable table;
  private NetworkTableEntry tx;
  private NetworkTableEntry ty;
  private NetworkTableEntry ta;
  private NetworkTableEntry tv;

  private double x; // Horizontal offset of crosshair from target
  private double y; // Vertical offset of crosshair from target
  private double v;
  private double area; // Amount of area target takes up on camera (0-100%)

  private double distanceToHoop;

  private double prevEroor;
  private double integralX;
  /** Creates a new Limelight. */
  public Limelight() {
    //turretFinderMotor = new WPI_TalonFX(Constants.TURRET_SPINNY_MOTOR);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
    table = NetworkTableInstance.getDefault().getTable("limelight") ;
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    tv = table.getEntry("tv");
    prevEroor = 0;
  }

  public double getHorizontalValue() {
    x = table.getEntry("tx").getDouble(0.0);
    disX = (x-1 < -29.8)? -29.8 : x-1;
    double calculated = (disX/450)*3;
    calculated = (Math.abs(calculated)<= Constants.TURRET_TOLERANCE) ? 0 : (calculated >= .2) ? .2 : calculated;
    return calculated;
  }

  @Override
  public void periodic() {
    updateVals();
    //tracking.calculate()
  }

  // public void setLEDMode (boolean enabled){
  //   int value = enabled ? 3 : 1;
  //   this.table.getEntry("ledMode").setNumber(value);
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
    v = tv.getDouble(0.0);
    
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
    if(x < Constants.TURRET_SPINNY_ERROR_MARGIN && x > Constants.TURRET_SPINNY_ERROR_MARGIN * -1){
      SmartDashboard.putBoolean("Locked on Target", true);
    }
    else if(x > Constants.TURRET_SPINNY_ERROR_MARGIN || x < Constants.TURRET_SPINNY_ERROR_MARGIN * -1){
      SmartDashboard.putBoolean("Locked on Target", false);
    }

    return distanceToHoop;
  }

  public double getX(){
    return x;
  }

  public boolean hasTarget(){
    return v == 1.0;
  }

  public void updateVals(){
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    v = tv.getDouble(0.0);
  }

}


//Old Code
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
  // public void stopTurryFindy() {
  //   turretFinderMotor.stopMotor();
  // }
  //public void runTurretFinder(XboxController controller){
  //  runTurretFinder(((controller.getRawAxis(Constants.RIGHT_TRIG))-(controller.getRawAxis(Constants.LEFT_TRIG))));
  // }
  // public void runTurretFinder(double vel) {
  //   turretFinderMotor.set(vel);
  // }  