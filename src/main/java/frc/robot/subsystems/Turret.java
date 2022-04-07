// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Turret extends SubsystemBase {
  private WPI_TalonFX turretMotor;
  private boolean trackingOn;
  private PIDController tracking;
  private double turretAngle = 0;
  private double setpoint = turretAngle;
  private ShuffleBoardConfig shuffle;
  //private double turretAngle;
  /** Creates a new Turret. */
  public Turret(ShuffleBoardConfig config) {
    shuffle = config;
    turretMotor = new WPI_TalonFX(Constants.TURRET_SPINNY_MOTOR);
    turretMotor.setInverted(true);
    trackingOn = false;
    turretMotor.setNeutralMode(NeutralMode.Brake);
    turretMotor.configFactoryDefault();
    turretMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 500);
    turretMotor.setSelectedSensorPosition(0);
    tracking = new PIDController(config.getTrackingData(1), config.getTrackingData(2), 0);
    tracking.setTolerance(3);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    turretAngle = getDegrees();
    turretMotor.set(MathUtil.clamp(tracking.calculate(turretAngle, MathUtil.clamp(setpoint, -90, 90)), -0.2, 0.2));
    SmartDashboard.putNumber("TurretAngle", turretAngle);
    SmartDashboard.putNumber("TurretSetpoint", setpoint);
  }

  //Spin the Turret
  public void spinTurret(double vel) {
     turretMotor.set(vel);
  }

  public void runTurretFinder(double setpoint) {
    this.setpoint = setpoint+turretAngle;
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

  public double getDegrees(){
    return turretMotor.getSelectedSensorPosition()/((112/15)*20480)*360;
  }
}
