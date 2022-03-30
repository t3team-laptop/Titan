// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ManualMovements.Turret;



import javax.swing.text.DefaultEditorKit.CutAction;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class AutoHood extends CommandBase {
  private Shooter shooty;
  private double targetPosition, currentPosition, error;
  private double marginOfError;
  private Limelight limy;
  private double distance;
  private double hoodyKp;
  private double hoodyI;
  private RelativeEncoder hoodEncoder;
  private boolean finish;
  /** Creates a new AdjustHood. */
  public AutoHood(Shooter shooty, Limelight limy, double targetPosition) {
    this.shooty = shooty;
    this.limy = limy;
    this.targetPosition = targetPosition;
    distance = limy.getDistanceToHoop();
    hoodEncoder = shooty.getHoodEncoder();
    marginOfError = Constants.HOOD_MOE;
    hoodyKp = Constants.HOOD_KP;
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
    currentPosition = hoodEncoder.getPosition();
    if(currentPosition < targetPosition + marginOfError){
      error = targetPosition - currentPosition;
      shooty.shooterHoodRun(error * Constants.AUTO_TURNING_KP + 0.1);
    }
    else if(currentPosition > targetPosition - marginOfError){
      error = targetPosition - currentPosition;
      shooty.shooterHoodRun(error * Constants.AUTO_TURNING_KP*-1 - 0.1);
    }
    else{
      finish = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooty.shooterHoodStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }

  //Was in execute
  //Equation to find targetPosition of hood based off distance - Add this here
    // while((hoodEncoder.getPosition() > targetPosition + marginOfError) || (hoodEncoder.getPosition() < targetPosition - marginOfError)){
    //   hoodyI = hoodEncoder.getPosition() - targetPosition;
    //   shooty.shooterHoodRun(hoodyKp * hoodyI);
    // }
    // shooty.shooterHoodRun(Constants.SHOOTER_HOOD_UP_SPEED);
}
