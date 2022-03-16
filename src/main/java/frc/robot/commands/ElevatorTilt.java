// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class ElevatorTilt extends CommandBase {
  private Elevator elevator;
  /** Creates a new ElevatorTilt. */
  public ElevatorTilt(Elevator elevator) {
    this.elevator = elevator;
    addRequirements(elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.getElevatorTiltMotor().configFactoryDefault();
    elevator.getElevatorTiltMotor().configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 5000);
    elevator.getElevatorTiltMotor().setSelectedSensorPosition(0, 0, 5000);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(elevator.getElevatorTiltMotor().getSelectedSensorPosition() < 25){
      elevator.elevatorTiltBack(Constants.ELEVATOR_TILT_SPEED);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.elevatorTiltStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
