// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.motorcontrol.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AutonomousPathDrivetrain extends SubsystemBase {
  public WPI_TalonFX leftFront = new WPI_TalonFX(Constants.LEFT_FRONT);
  public WPI_TalonFX leftBack = new WPI_TalonFX(Constants.LEFT_BACK);
  public WPI_TalonFX rightFront = new WPI_TalonFX(Constants.RIGHT_FRONT);
  public WPI_TalonFX rightBack = new WPI_TalonFX(Constants.RIGHT_BACK);
  private final MotorControllerGroup leftMotors = new MotorControllerGroup(leftFront, leftBack);
  private final MotorControllerGroup rightMotors = new MotorControllerGroup(rightFront, rightBack);
  private final DifferentialDrive drive = new DifferentialDrive(leftMotors, rightMotors);
  private final DifferentialDriveOdometry m_odometry;
  public ADIS16470_IMU gyro = new ADIS16470_IMU();
  private static boolean tank, arcade, gta, superDrive;
    
  /** Creates a new DriveTrain. */
  public AutonomousPathDrivetrain() {
    arcade = true;

    leftMotors.setInverted(true);    
    rightMotors.setInverted(false);     

    leftFront.configFactoryDefault();
    rightFront.configFactoryDefault();
    leftFront.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 500);
    rightFront.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 500);
    leftFront.setSelectedSensorPosition(0, 0, 500);
    rightFront.setSelectedSensorPosition(0, 0, 500);

    m_odometry = new DifferentialDriveOdometry(getRotation2d());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Talon's don't have get distance so find the equivalent method for them
    // m_odometry.update(
    //     gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
  }

  // Find the "k" numbers that would work with our talons
  // private double nativeUnitsToDistanceMeters(double sensorCounts){
  //   double motorRotations = (double)sensorCounts / kCountsPerRev;
  //   double wheelRotations = motorRotations / kGearRatio;
  //   double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches));
  //   return positionMeters;
  // }

  // I don't think talon's have getRate so find the equivalent
  // public DifferentialDriveWheelSpeeds getWheelSpeeds() {
  //   return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  // }

  public void driveArcadeDrive(XboxController controller,double speed){
    drive.arcadeDrive(controller.getRawAxis(Constants.LEFT_JOY_Y)*speed, controller.getRawAxis(Constants.RIGHT_JOY_X)*0.65*-1, true);
  }

  public void stop(){
    drive.stopMotor();
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void driveArcade(double speed, double rotation){
    drive.arcadeDrive(speed, rotation);
  }
  
  public void driveForward(double speed){
    drive.tankDrive(speed, speed);
  }

  public static int getPov(XboxController controller){
    return controller.getPOV(Constants.JOY_POV);
  }

  public Rotation2d getRotation2d(){
    Rotation2d rot = new Rotation2d();
    return Rotation2d.fromDegrees(-gyro.getAngle());
  }
}