// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Turret;

public class CenterTarget extends CommandBase {
  private Turret turret;
  private Limelight limelight;

    /**
     *
     * @param turret turret subsystem
     * @param vision vision subsystem
     */
    public CenterTarget(Turret turret, Limelight limy) {
        this.turret = turret;
        this.limelight = limy;
        addRequirements(turret, limelight);
    }

    @Override
    public void execute() {
        if (this.turret.getTrackingSwitch() && limelight.hasTarget()) {
            this.turret.runTurretFinder(limelight.getHorizontalValue());;
            // this.turret.turretBrakeMode(false);
        } else {
            this.turret.runTurretFinder(0);
            // this.turret.turretBrakeMode(true);
        }
        this.limelight.setLEDMode(this.turret.getTrackingSwitch());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
