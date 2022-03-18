// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.Constants;
//test

public class LocateHoop extends CommandBase {
  private Limelight limy; 
  private WPI_TalonFX turretMotor;
  private double turretSpeed;
  private double minTurretSpeed;
  private double Kp;
  private double heading_error;
  private double turnTurretKp, turnTurretI; // P & I in PID for fixing turret rotation when maxed
  private boolean lastOffsetRight;
  private boolean lockedOn;
  private int P, I, D;
  private int integral, previous_error, setpoint;
  private double turnSpeed;
  /** Creates a new limyBall. */
  public LocateHoop(Limelight limy) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.limy = limy;
    turretMotor = limy.turretFinderMotor;
    P = 1;
    I = 1;
    D = 1;
    integral = 0;
    previous_error = 0;
    setpoint = 0;
    turnSpeed = 0;
    addRequirements(limy);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turretSpeed = Constants.TURRET_ADJUST_SPEED;
    minTurretSpeed = Constants.MINIMUM_TURRET_ADJUST_SPEED;
    Kp = Constants.KP;
    turnTurretKp = Constants.TURN_TURRET_KP;
    heading_error = -limy.getX();
    lastOffsetRight = false; // which direction to turn based on where the limelight was last seen
    lockedOn = false;
    turretMotor.configFactoryDefault();
    turretMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 5000);
    turretMotor.setSelectedSensorPosition(0, 0, 5000);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(sensorToDegrees() >= 90){
      turnTurretI = sensorToDegrees(); 
      while(!(turnTurretI < 5 && turnTurretI > -5)){
        limy.runTurretFinder(turnTurretKp * turnTurretI); // if this doesn't work maybe add or subtract a minimum speed that the motors need to run
      }
    }
    else if (sensorToDegrees() <= -90){
      turnTurretI = sensorToDegrees(); 
      while(!(turnTurretI < 5 && turnTurretI > -5)){
        limy.runTurretFinder(turnTurretKp * turnTurretI); // if this doesn't work maybe add or subtract a minimum speed that the motors need to run
      }
    }
    else{      
      limy.runTurretFinder(turnTurretSpeed());
    }
    System.out.println("work");
    SmartDashboard.putNumber("Turret Encoder Position", turretMotor.getSelectedSensorPosition());
    SmartDashboard.putBoolean("Locked On Targer", lockedOn);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    limy.stopTurryFindy();
  }

  private double sensorToDegrees(){
    return turretMotor.getSelectedSensorPosition() / Constants.TALON_COUNTSPERREV * 360;
  }

  private double turnTurretSpeed(){
    if(limy.getX() > 0.0){
      lastOffsetRight = true;
    }
    else{
      lastOffsetRight = false;
    }

    heading_error = -limy.getX();
    // if (!limy.hasTarget())
    // {
    //     // We don't see the target, seek for the target by spinning in place at a safe speed.
    //     if(lastOffsetRight){
    //       turretSpeed = Constants.TURRET_ADJUST_SPEED;
    //     }
    //     else if(!lastOffsetRight){
    //       turretSpeed = Constants.TURRET_ADJUST_SPEED * -1;
    //     }
    // }
    //else
    if(limy.hasTarget())
    {
      //Check that we do need the 1.0 for each side
      // We do see the target, execute aiming code
      if (limy.getX() > Constants.TURRET_SPINNY_ERROR_MARGIN)
      {
              turretSpeed = Kp*heading_error - minTurretSpeed;
      }
      else if (limy.getX() < Constants.TURRET_SPINNY_ERROR_MARGIN * -1)
      {
              turretSpeed = Kp*heading_error + minTurretSpeed;
      }
      else{
        turretSpeed = 0.0;
        lockedOn = true;
      }
    }
    return turretSpeed;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
