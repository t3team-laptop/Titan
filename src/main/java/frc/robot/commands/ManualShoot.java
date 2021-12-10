// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;

public class ManualShoot extends CommandBase {
  /** Creates a new ManualShoot. */
  private final Shooter shooty;
  private static double manSpeed, temp;
  private static int count = 1;
  private static int pov = -1;
  public ManualShoot(Shooter s) {
    // Use addRequirements() here to declare subsystem dependencies.
    shooty = s;
    manSpeed = 0.5;
    addRequirements(shooty);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pov = DriveTrain.getPov(RobotContainer.driverJoystick);
      if(pov == 0){
        if(manSpeed < 1){
          temp += 0.1;
          count++;
        }
      }else if(pov == 180){
        if(manSpeed > 0){
          temp -= 0.1;
          count++;
        }
      }else if(pov == 90){
        shooty.shooterRevUP(-1*manSpeed);
        System.out.println(manSpeed);
      }else{
        if(temp != 0){
          manSpeed += temp/count;
          count = 0;
          temp = 0;
          manSpeed = (double)Math.round(manSpeed*10)/10;
        }
        shooty.stop();
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooty.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
