// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import frc.robot.Constants;

public class BallSuck extends SubsystemBase {
  /** Creates a new BallSuck. */
  private WPI_TalonSRX leftSuck, rightSuck;
  private SpeedControllerGroup ballSuck;
  public BallSuck() {
    leftSuck = new WPI_TalonSRX(Constants.LEFT_BALLSUCK);
    rightSuck = new WPI_TalonSRX(Constants.RIGHT_BALLSUCK);
    leftSuck.setInverted(true);
    ballSuck = new SpeedControllerGroup(leftSuck, rightSuck);
  }
  public void ballSuckGo() {ballSuck.set(Constants.BALLSUCK);}
  public void ballSuckStop() {ballSuck.set(0.0);}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
