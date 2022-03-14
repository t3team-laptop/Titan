// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;



import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class AdjustHood extends CommandBase {
  Shooter shooty;
  Limelight limy;
  private double distance;
  private double hoodyKp;
  /** Creates a new AdjustHood. */
  public AdjustHood(Shooter shooty, Limelight limy) {
    this.shooty = shooty;
    this.limy = limy;
    distance = limy.getDistanceToHoop();
    addRequirements(shooty, limy);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {    
    shooty.shooterHoodRun(Constants.SHOOTER_HOOD_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooty.shooterHoodStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}