// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import com.qualcomm.robotcore.hardware;
public class ShootBall extends CommandBase implements DcMotorEx{
  private Shooter shooty;
  private Limelight limy;
  private double distance;
  private double hoodSpeed;
  private CANSparkMax hoodController;
  private RelativeEncoder hoodEncoder;
  private DcMotorEx hoodMotor;
  public ShootBall(Shooter shooty, Limelight limy) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooty = shooty;
    this.limy = limy;
    addRequirements(shooty, limy);
    hoodMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hoodSpeed = Constants.SHOOTER_HOOD_SPEED;
    hoodController = new CANSparkMax(Constants.SHOOTER_HOOD_PITCH,  MotorType.kBrushless);
    hoodController.restoreFactoryDefaults();
    hoodEncoder = hoodController.getEncoder();
    hoodController.setMode(CANSparkMax.RunMode.STOP_AND_RESET_ENCODER);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    distance = limy.getDistanceToHoop();
    shooty.shooterHoodRun();
    shooty.shootyLaunchyRun();
    SmartDashboard.putNumber("Hood Position", hoodEncoder.getPosition());
    hoodController.setMode();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooty.shooterHoodStop();
    shooty.shootyLaunchyIdle();
    shooty.shootyLaunchyStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
