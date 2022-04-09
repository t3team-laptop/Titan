// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ManualMovements.Turret;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Indexing;
import frc.robot.subsystems.Shooter;

public class SafeShot extends CommandBase {
  /** Creates a new SafeShot. */
  private Shooter shooty;
  private Indexing indexy;
  private XboxController controly;
  public SafeShot(Shooter shoot, Indexing index, XboxController control) {
    // Use addRequirements() here to declare subsystem dependencies.
    shooty = shoot;
    indexy = index;
    controly = control;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!(controly.getRawAxis(Constants.LEFT_TRIG) >= 0.6)){
      shooty.shootySuckyRun(Constants.SHOOTER_SUCK_SPEED*-0.75);
      indexy.moveForward();
    }else{
      indexy.indexingStop();
      shooty.shootySuckyRun(Constants.SHOOTER_SUCK_SPEED);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexy.indexingStop();
    shooty.shootySuckyStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
