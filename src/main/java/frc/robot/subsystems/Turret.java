// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Turret extends SubsystemBase {
  private WPI_TalonFX turretMotor;
  private boolean trackingOn;
  /** Creates a new Turret. */
  public Turret() {
    turretMotor = new WPI_TalonFX(Constants.TURRET_SPINNY_MOTOR);
    turretMotor.setInverted(true);
    trackingOn = false;
    turretMotor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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
