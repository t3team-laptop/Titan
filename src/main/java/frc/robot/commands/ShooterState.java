// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class ShooterState extends CommandBase {
  /** Creates a new ShooterState. */
  private final Shooter shooty;
  private final Limelight limy;
  public ShooterState(Shooter s, Limelight l) {
    // Use addRequirements() here to declare subsystem dependencies.
    shooty = s;
    limy = l;
    addRequirements(shooty);
    addRequirements(limy);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      if(Shooter.getState()){
        shooty.shooterRevUP(Constants.SHOOTER_IDLE);
      }else{
        shooty.shooterRevUP(limy.getPercentage());
      }
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return false;
    
  }
}
