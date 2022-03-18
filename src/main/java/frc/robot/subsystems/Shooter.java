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
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Shooter extends PIDSubsystem {
  public WPI_TalonSRX shootySucky;
  public WPI_TalonFX shootyLaunchy;
  public CANSparkMax shooterHood;
  public RelativeEncoder hoodEncoder;
  private SimpleMotorFeedforward shooterFeed; 
  public Shooter() {
    super(new PIDController(Constants.SHOOTER_LAUNCH_KP, Constants.SHOOTER_LAUNCH_KI, Constants.SHOOTER_LAUNCH_KD));
    shootySucky = new WPI_TalonSRX(Constants.SHOOTER_SUCK_MOTOR);
    shootySucky.setInverted(true);
    shooterFeed = new SimpleMotorFeedforward(Constants.kSVolts, Constants.kVVoltSecondsPerRotation);
    shooterHood = new CANSparkMax(Constants.SHOOTER_HOOD_PITCH,  MotorType.kBrushless);
    //hoodEncoder = shooterHood.getEncoder();
    shootyLaunchy = new WPI_TalonFX(Constants.SHOOTER_LAUNCH_MOTOR);
    shootyLaunchy.setInverted(true);    
    shootyLaunchy.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 1, 1);
    getController().setTolerance(Constants.SHOOTER_LAUNCH_TOLERANCE);
    setSetpoint(0);
  }

  @Override
  public void useOutput(double output, double setpoint) {
      shootyLaunchy.setVoltage(output + shooterFeed.calculate(setpoint));
  }

  @Override
  public double getMeasurement() {
      double selSenVel = shootyLaunchy.getSelectedSensorVelocity(0);
      double rotPerSec = (double) selSenVel / Constants.kUnitsPerRevolution * 10; /* scale per100ms to perSecond */

      // System.out.println("SHOOTER RPM (Speed): " + rotPerSec * 60);
      // System.out.println("Voltage: " + shooter.getMotorOutputVoltage());
      return rotPerSec;
  }  

  @Override
  public void periodic() {
    if (m_enabled) {
      useOutput(m_controller.calculate(getMeasurement(), getSetpoint()), getSetpoint());
    }
    // This method will be called once per scheduler run
  }

  public RelativeEncoder getHoodEncoder(){
    return shooterHood.getEncoder();
  }

  //Runs and stops the motors
  public void shootySuckyRun(double speed){
    shootySucky.set(speed);
  }
  public void shootySuckyStop(){
    shootySucky.stopMotor();
  }

  public void shooterHoodRun(double speed){
    shooterHood.set(speed);
  }
  public void shooterHoodStop(){
    shooterHood.stopMotor();
  }

  public void shootyLaunchyRun(double speed){
    shootyLaunchy.set(speed);
    //shootyLaunchy.set(Contro, value);)
  }
  public void shootyLaunchyIdle(){
    shootyLaunchy.set(Constants.SHOOTER_IDLE_SPEED);
  }
  public void shootyLaunchyStop(){
    shootyLaunchy.stopMotor();
  }

  //Rosbots Code
  /**
     * Get RPM of the Shooter motor
     *
     * @return RPM of the Shooter Motor
     */
    public double getRPM() {
      return getMeasurement() * 60;
  }

  /**
   * Checks if Shooter is within tolerance of the setpoint
   *
   * @return True if shooter is at the correct speed
   */
  public boolean atSetpoint() {
      return m_controller.atSetpoint();
  }

  /**
   * Enable the Shooter Subsystem
   */
  public void enableShooter() {
      this.enable();
  }
}
