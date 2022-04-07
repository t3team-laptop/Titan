// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ManualMovements.Turret;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Turret;

public class ManualSpinTurret extends CommandBase {
  // WPI_TalonSRX turretMotor;
  // boolean lefty;
  // XboxController controller;
  /** Creates a new runTurret. */
  private Turret turret;
  private boolean lefty;
  private double multiplier;
  private XboxController sControl;
  public ManualSpinTurret(Turret turret, XboxController sc){
    // this.turretMotor = turretMotor;
    // this.lefty = lefty;
    // this.controller = controller;
    this.turret = turret;
    sControl = sc;
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if(lefty){
    //   //System.out.println("Moving turret lefty");
    //   turret.spinTurret(Constants.MANUAL_TURRET_SPEED * -1 * multiplier);
    // }
    // else if (!lefty){
    //   //System.out.println("Moving turret righty");
    //   turret.spinTurret(Constants.MANUAL_TURRET_SPEED * multiplier);
    // }
    turret.spinTurret(sControl.getRawAxis(Constants.LEFT_JOY_X)*0.25);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.stopTurryFindy();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
