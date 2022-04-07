// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.Map;

public class ShuffleBoardConfig extends SubsystemBase {
  /** Creates a new ShuffleBoard. */
  private ShuffleboardTab tab;
  private NetworkTableEntry rpm;
  private NetworkTableEntry rpmKp;
  private NetworkTableEntry rpmKi;

  private NetworkTableEntry hoodSpeed;
  private NetworkTableEntry hoodPos;
  private NetworkTableEntry hoodKp;
  private NetworkTableEntry hoodKi;

  private NetworkTableEntry trackingSpeed;
  private NetworkTableEntry trackingKp;
  private NetworkTableEntry trackingKi;
  private NetworkTableEntry trackingAngle;

  public ShuffleBoardConfig() {
    tab = Shuffleboard.getTab("Testing");

    rpm = tab.add("RPM", Constants.SHOOTER_LAUNCH_SPEED_CLOSE).withSize(2, 1).withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", 0)).getEntry();
    rpmKp = tab.add("RPMkp", Constants.SHOOTER_LAUNCH_KP).withSize(1, 1).withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", 0)).getEntry();
    rpmKi = tab.add("RPMki", Constants.SHOOTER_LAUNCH_KI).withSize(1, 1).withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", 0)).getEntry();

    hoodSpeed = tab.add("hoodSpeed", Constants.HOOD_SPEED).withSize(2, 1).withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", 0)).getEntry();
    hoodPos = tab.add("hoodPos", 0).withSize(1, 1).withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", 0)).getEntry();
    hoodKp = tab.add("hoodKp", Constants.HOOD_KP).withSize(1, 1).withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", 0)).getEntry();
    hoodKi = tab.add("hoodKi", Constants.HOOD_KI).withSize(1, 1).withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", 0)).getEntry();

    trackingSpeed = tab.add("trackingSpeed", Constants.TURRET_ADJUST_SPEED).withSize(2, 1).withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", 0)).getEntry();
    trackingKp = tab.add("trackingKp", Constants.TURRETXP).withSize(1, 1).withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", 0)).getEntry();
    trackingKi = tab.add("trackingKi", Constants.TURRETXI).withSize(1, 1).withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", 0)).getEntry();
    trackingAngle = tab.add("trackingAngle", Constants.TURRET_TOLERANCE).withSize(1, 1).withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", 0)).getEntry();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getRPMData(int item){
    switch (item){
      case(0):
        return rpm.getDouble(0);
      case(1):
        return rpmKp.getDouble(0);
      case(2):
        return rpmKi.getDouble(0);
    }
    return 0;
  }

  public double getHoodData(int item){
    switch (item){
      case(0):
        return hoodSpeed.getDouble(0);
      case(1):
        return hoodPos.getDouble(0);
      case(2):
        return hoodKp.getDouble(0);
      case(3):
        return hoodKi.getDouble(0);
    }
    return 0;
  }

  public double getTrackingData(int item){
    switch (item){
      case(0):
        return trackingSpeed.getDouble(0);
      case(1):
        return trackingKp.getDouble(0);
      case(2):
        return trackingKi.getDouble(0);
      case(3):
        return trackingAngle.getDouble(0);
    }
    return 0;
  }
}
