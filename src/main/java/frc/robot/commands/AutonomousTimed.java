// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;

public class AutonomousTimed extends CommandBase {
  private DriveTrain driveTrain;
  private boolean finish = false;
  private Timer timer;
  private double initialTime;
  private Shooter shoot;

  /** Creates a new DriveForwardTimed. */
  public AutonomousTimed(DriveTrain dt, Shooter shoot){
    driveTrain = dt;
    this.shoot = shoot;
    addRequirements(driveTrain, shoot);
    timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialTime = Timer.getFPGATimestamp();
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Timer.getFPGATimestamp() - initialTime < 2.0){
      //shoot.shootyLaunchyRun(0.6);
      shoot.shootySuckyRun(Constants.SHOOTER_SUCK_SPEED);
    }
    else if(Timer.getFPGATimestamp() - initialTime > 2.0 && Timer.getFPGATimestamp() - initialTime < 2.5){
      shoot.shootyLaunchyStop();
      shoot.shootySuckyStop();
    }
    else if(Timer.getFPGATimestamp() - initialTime > 2.5 && Timer.getFPGATimestamp() - initialTime < 4.5){
      driveTrain.driveForward(Constants.AUTONOMOUS_SPEED);
    }
    else if (Timer.getFPGATimestamp() - initialTime > 5.5){
      driveTrain.driveForward(0.0);
      finish = true;
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
