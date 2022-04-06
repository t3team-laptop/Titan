// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Turret extends SubsystemBase {
  private WPI_TalonFX turretMotor;
  private boolean trackingOn;
  //private double turretAngle;
  /** Creates a new Turret. */
  public Turret() {
    turretMotor = new WPI_TalonFX(Constants.TURRET_SPINNY_MOTOR);
    turretMotor.setInverted(false);
    trackingOn = false;
    turretMotor.setNeutralMode(NeutralMode.Brake);
    turretMotor.configFactoryDefault();
    turretMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 500);
    turretMotor.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("TurretAngle", turretMotor.getSelectedSensorPosition());
  }

  //Spin the Turret
  public void runTurretFinder(double vel) {
    turretMotor.set(vel);
  }

  //Stop Spinning the turret
  public void stopTurryFindy() {
    turretMotor.stopMotor();
  }

  //Toggle tracking variable
  public void toggleTracking(){
    trackingOn = !trackingOn;
  }

  //Return whether or not we are tracking
  public boolean getTrackingOn(){
    return trackingOn;
  }
}
