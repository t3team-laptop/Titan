// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ManualMovements.Turret;

import edu.wpi.first.math.MathUtil;
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
  private boolean lefty, axis;
  private double multiplier;
  private XboxController sControl;
  private double minSpeed = -0., maxSpeed = 0.3;
  public ManualSpinTurret(Turret turret, XboxController sc, double mult, boolean axis){
    // this.turretMotor = turretMotor;
    // this.lefty = lefty;
    // this.controller = controller;
    this.multiplier = mult;
    this.turret = turret;
    this.axis = axis;
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
    //   ////System.out.println("Moving turret lefty");
    //   turret.spinTurret(Constants.MANUAL_TURRET_SPEED * -1 * multiplier);
    // }
    // else if (!lefty){
    //   ////System.out.println("Moving turret righty");
    //   turret.spinTurret(Constants.MANUAL_TURRET_SPEED * multiplier);
    // }
    minSpeed = (turret.getDegrees() > -120) ? -0.3: 0;
    maxSpeed = (turret.getDegrees() < 120) ? 0.3 : 0;
    turret.spinTurret(MathUtil.clamp(sControl.getRawAxis(Constants.LEFT_JOY_X)*0.3 + sControl.getRawAxis(Constants.RIGHT_JOY_X)*multiplier, minSpeed, maxSpeed));
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
