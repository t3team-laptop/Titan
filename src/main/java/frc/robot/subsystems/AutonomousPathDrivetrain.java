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
  public WPI_TalonFX leftFront;
  public WPI_TalonFX leftBack;
  public WPI_TalonFX rightFront;
  public WPI_TalonFX rightBack;
  private final MotorControllerGroup leftMotors;
  private final MotorControllerGroup rightMotors;
  private final DifferentialDrive drive;
  private final DifferentialDriveOdometry m_odometry;
  public ADIS16470_IMU gyro = new ADIS16470_IMU();
  private static boolean tank, arcade, gta, superDrive;
    
  /** Creates a new DriveTrain. */
  public AutonomousPathDrivetrain() {
    arcade = true;
    leftFront = new WPI_TalonFX(Constants.LEFT_FRONT);
    leftFront.setInverted(true);
    leftBack = new WPI_TalonFX(Constants.LEFT_BACK);
    leftBack.setInverted(true);
    rightFront = new WPI_TalonFX(Constants.RIGHT_FRONT);
    rightFront.setInverted(false);
    rightBack = new WPI_TalonFX(Constants.RIGHT_BACK);
    rightBack.setInverted(false);

    leftMotors = new MotorControllerGroup(leftFront, leftBack);
    rightMotors = new MotorControllerGroup(rightFront, rightBack);
    drive = new DifferentialDrive(leftMotors, rightMotors);

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
    m_odometry.update(
        getRotation2d(), nativeUnitsToDistanceMeters(leftFront.getSelectedSensorPosition()), nativeUnitsToDistanceMeters(rightFront.getSelectedSensorPosition()));
  }

  //One I might be the same as setDistancePerPulse the other might be getDistance, or maybe you only need getDistance
  // Find the "k" numbers that would work with our talons
  private double nativeUnitsToDistanceMeters(double sensorCounts){
    double motorRotations = (double)sensorCounts / Constants.kCountsPerRev;
    double wheelRotations = motorRotations / Constants.kGearRatio;
    double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(Constants.kWheelRadiusInches));
    return positionMeters;
  }

  private int distanceToNativeUnits(double positionMeters){
    double wheelRotations = positionMeters/(2 * Math.PI * Units.inchesToMeters(Constants.kWheelRadiusInches));
    double motorRotations = wheelRotations * Constants.kGearRatio;
    int sensorCounts = (int)(motorRotations * Constants.kCountsPerRev);
    return sensorCounts;
  }

  // I don't think talon's have getRate so find the equivalent
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(nativeUnitsToDistanceMeters(leftFront.getSelectedSensorVelocity()) * 10, nativeUnitsToDistanceMeters(rightFront.getSelectedSensorVelocity()) * 10);
  }

  public void driveArcadeDrive(XboxController controller,double speed){
    drive.arcadeDrive(controller.getRawAxis(Constants.LEFT_JOY_Y)*speed, controller.getRawAxis(Constants.RIGHT_JOY_X)*0.65*-1, true);
  }

  public void stop(){
    drive.stopMotor();
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetEncoders() {
    leftFront.setSelectedSensorPosition(0, 0, 500);
    rightFront.setSelectedSensorPosition(0, 0, 500);
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, getRotation2d());
  }

  public double getAverageEncoderDistance() {
    return (nativeUnitsToDistanceMeters(leftFront.getSelectedSensorPosition()) + nativeUnitsToDistanceMeters(rightFront.getSelectedSensorPosition())) / 2.0;
  }

  public void arcadeDrive(double fwd, double rot) {
    drive.arcadeDrive(fwd, rot);
  }
  
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMotors.setVoltage(leftVolts);
    rightMotors.setVoltage(rightVolts);
    drive.feed();
  }

  public static int getPov(XboxController controller){
    return controller.getPOV(Constants.JOY_POV);
  }

  public Rotation2d getRotation2d(){
    Rotation2d rot = new Rotation2d();
    return Rotation2d.fromDegrees(-gyro.getAngle());
  }

  public void setMaxOutput(double maxOutput) {
    drive.setMaxOutput(maxOutput);
  }
  
  public void zeroHeading() {
    gyro.reset();
  }

  public double getHeading() {
    return gyro.getAngle();
  }

  public double getTurnRate() {
    return -gyro.getRate();
  }
}