// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private WPI_TalonFX shooterMotor1, shooterMotor2;
  private SpeedControllerGroup shooterMotors;
  private static boolean state = false;
  private static boolean manual = true;
  /** Creates a new Shooter. */
  public Shooter() {
    shooterMotor1 = new WPI_TalonFX(Constants.GLOCK1);
    shooterMotor2 = new WPI_TalonFX(Constants.GLOCK2);
    shooterMotors = new SpeedControllerGroup(shooterMotor1, shooterMotor2);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void shooterRevUP(double speed){
    shooterMotors.set(speed);
  }
  public void stop(){
    shooterMotors.stopMotor();
  }
  public static void stateToggle(){
    state = !state;
  }
  public static boolean getState(){
    return state;
  }
  public static boolean getManual(){
    return manual;
  }
}
