// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


<<<<<<< HEAD
public class Shooter extends PIDSubsystem {
  public BuiltInWidgets kPIDCommand;
  public WPI_TalonSRX shootySucky;
  public WPI_TalonFX shootyLaunchy;
  private SimpleMotorFeedforward shooterFeed; 
  private NetworkTableEntry shuffleboardShit;
  private ShuffleboardTab shooterTab;
  public Shooter() {
    super(new PIDController(Constants.SHOOTER_LAUNCH_KP, Constants.SHOOTER_LAUNCH_KI, Constants.SHOOTER_LAUNCH_KD));
    shooterTab = Shuffleboard.getTab("Shooter PID");
    shootySucky = new WPI_TalonSRX(Constants.SHOOTER_SUCK_MOTOR);
    shootySucky.setInverted(true);
    shooterFeed = new SimpleMotorFeedforward(Constants.kSVolts, Constants.kVVoltSecondsPerRotation);
    shootyLaunchy = new WPI_TalonFX(Constants.SHOOTER_LAUNCH_MOTOR);
    shootyLaunchy.setInverted(true);    
    shootyLaunchy.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 1, 1);
    getController().setTolerance(Constants.SHOOTER_LAUNCH_TOLERANCE);
    setSetpoint(0);
    shuffleboardShit = Shuffleboard.getTab("Shooter PID").addPersistent("KP", Constants.SHOOTER_LAUNCH_KP).withWidget(BuiltInWidgets.kPIDCommand).getEntry();
    shuffleboardShit = Shuffleboard.getTab("Shooter PID").addPersistent("KI", Constants.SHOOTER_LAUNCH_KI).withWidget(BuiltInWidgets.kPIDCommand).getEntry();
    shuffleboardShit = Shuffleboard.getTab("Shooter PID").addPersistent("KD", Constants.SHOOTER_LAUNCH_KD).withWidget(BuiltInWidgets.kPIDCommand).getEntry();
=======
public class Shooter extends SubsystemBase {
  public WPI_TalonSRX shootySucky;
  public WPI_TalonFX shootyLaunchy;
  public CANSparkMax shooterHood;
  public RelativeEncoder hoodEncoder;
  public Shooter() {
    shootySucky = new WPI_TalonSRX(Constants.SHOOTER_SUCK_MOTOR);
    shootySucky.setInverted(true);
    shooterHood = new CANSparkMax(Constants.SHOOTER_HOOD_PITCH,  MotorType.kBrushless);
    //hoodEncoder = shooterHood.getEncoder();
    shootyLaunchy = new WPI_TalonFX(Constants.SHOOTER_LAUNCH_MOTOR);
    shootyLaunchy.setInverted(true);  
>>>>>>> 03c27405fcc6da66c7639b1daaf02e003ca12cc1
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //Runs and stops the motors
  public void shootySuckyRun(double speed){
    shootySucky.set(speed);
  }
  public void shootySuckyStop(){
    shootySucky.stopMotor();
  }

  // public void shootyLaunchyRun(double speed){
  //   shootyLaunchy.set(speed);
  //   //shootyLaunchy.set(Contro, value);)
  // }
  public void shootyLaunchyIdle(){
    m_controller.setSetpoint(Constants.SHOOTER_IDLE_SPEED);
  }
  public void shootyLaunchyStop(){
    this.disable();
    this.shootyLaunchy.stopMotor();
  }
}
