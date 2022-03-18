// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.Math;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight extends SubsystemBase {
  //public WPI_TalonFX turretFinderMotor;
  private NetworkTable table;
  private NetworkTableEntry tx;
  private NetworkTableEntry ty;
  private NetworkTableEntry ta;
  private NetworkTableEntry tv;

  private double x; // Horizontal offset of crosshair from target
  private double y; // Vertical offset of crosshair from target
  private double v;
  private double area; // Amount of area target takes up on camera (0-100%)

  private double horizontalError;

  private double threshold;
  private double Kp;
  private double targetAngleX;
  private double targetAngleY;
  private double prevEroor;
  private double turnSpeed;
  private double integralX;
  private ShuffleboardTab tab;
  private double disX;
  /** Creates a new Limelight. */
  public Limelight() {
    tab = Shuffleboard.getTab("Limy");
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = tab.add("tx", table.getEntry("tx")).getEntry();
    ty = tab.add("ty", table.getEntry("ty")).getEntry();;
    ta = table.getEntry("ta");
    tv = table.getEntry("tv");
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

  // public double PID(){
  //   double error = getHorizontalError() - Constants.LIMELIGHT_MOUNTING_ANGLE_DEGREES;
  //   this.integralX += (error*.02);
  //   double derivative = (error - this.prevEroor)/.2;
  //   return Constants.TURRETXP*error + Constants.TURRETXI*this.integralX + Constants.TURRETXD*derivative;
  // }

  public double getDistance(){
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
    double hoopHeight = 106.0;

    double angleToGoalDegrees = limelightMountAngleDegrees + y;
    double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);

    //calculate distance
    horizontalError = (hoopHeight - limelightLensHeight)/Math.tan(angleToGoalRadians);

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

    return horizontalError;
  }

  

  public double getX(){
    return x;
  }

  public boolean hasTarget(){
    return v == 1.0;
  }

  //public void runTurretFinder(XboxController controller){
  //  runTurretFinder(((controller.getRawAxis(Constants.RIGHT_TRIG))-(controller.getRawAxis(Constants.LEFT_TRIG))));
  // }
  public void setLEDMode(boolean enabled) {
    int value = enabled ? 3 : 1;
    this.table.getEntry("ledMode").setNumber(value);
  }

  public double getHorizontalValue() {
    // Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees | LL2: -29.8
    // to 29.8 degrees)
    x = table.getEntry("tx").getDouble(0.0);
    // targetFound = false;
    // disX = 0;
    disX = x;
    double calculated = (disX / 125) * 3;
    calculated = (Math.abs(calculated) <= Constants.TURRET_TOLERANCE) ? 0 : (calculated >= .3) ? .3 : calculated;
    return calculated;
  }
}