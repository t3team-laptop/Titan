// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Toggles;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;


public class ToggleDrive extends CommandBase {
  /** Creates a new ToggleDrive. */
  private static DriveTrain driveTrain;
  private static int pov = -1;
  public ToggleDrive(DriveTrain dt) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveTrain = dt;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pov = DriveTrain.getPov(RobotContainer.driverJoystick);
    if(pov == 270){
      //driveTrain.setTank();
      System.out.println("Tank drive activated.");
    }else if(pov == 0){
      //driveTrain.setArcade();
      System.out.println("Arcade drive activated.");
    }else if(pov == 90){
      //driveTrain.setGta();
      System.out.println("GTA drive activated.");
    }
    else if(pov == 180) {
      //driveTrain.setSuper();
      System.out.println("Super drive activated.");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
