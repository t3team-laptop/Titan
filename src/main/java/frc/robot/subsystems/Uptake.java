// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Uptake extends SubsystemBase {
  /** Creates a new Uptake. */
  public Uptake() {}
  WPI_TalonSRX uptake = new WPI_TalonSRX(Constants.UPTAKE);

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void uptakeGo(boolean reverse) {
    if(reverse){
      uptake.set(Constants.UPTAKE_SPEED * -1);
    }
    else{
      uptake.set(Constants.UPTAKE_SPEED);
    }
  }
  public void uptakeStop() {uptake.set(0.0);}
}
