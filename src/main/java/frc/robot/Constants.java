// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    //Drivetrain Motor IDs
    public static final int LEFT_FRONT = 4; //2022 is 4
    public static final int LEFT_BACK = 3; //2022 is 3
    public static final int RIGHT_FRONT = 2; //2022 is 2
    public static final int RIGHT_BACK = 1; //2022 is 1
    //Drivetrain function Constants
    public static final double DRIVETRAINSPEED = 0.75;
    public static final double DRIVE_FORWARD_TIME = 3.0;

    //Autonomous Path Planning Variables - change when you have the numbers from SysId
    public static final double ksVolts = 0.17345;
    public static final double kvVoltSecondsPerMeter = 0.94931;
    public static final double kaVoltSecondsSquaredPerMeter = 0.26554;
    public static final double kPDriveVel = 1.3576;

    public static final double kTrackwidthMeters = 0.61;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final double kMaxSpeedMetersPerSecond = 4;
    public static final double kMaxAccelerationMetersPerSecondSquared = 2.75;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    //Other Auto Constants And Stuff
    public static final int kCountsPerRev = 2048;
    public static final double kGearRatio = 4.286;
    public static final double kWheelRadiusInches = 3;
    public static final int k100msPerSecond = 10;



    //Indexing/Intake Motor IDs
    public static final int INDEX_LEFT = 11; //Indexing Left
    public static final int INDEX_RIGHT = 12; //Indexing Right
    public static final int INTAKE_MOTOR = 24;
    public static final int INTAKE_MOVE_MOTOR = 21;

    //Indexing/Intake function Constants
    public static final double INDEX_SPEED = -1;
    public static final double INTAKE_SPEED = -1;
    public static final double INTAKE_MOVE_SPEED_DOWN = .2;
    public static final double INTAKE_MOVE_SPEED_UP = .05;
    public static final double INTAKE_MOVEUP_TIME1 = 1.0;
    public static final double INTAKE_MOVEUP_TIME2 = .2;
    public static final double INTAKE_MOVEDOWN_TIME1 = 0.2;
    public static final double INTAKE_MOVEDOWN_TIME2 = 0.5;

    //Shooter Motor ID's
    public static final int TURRET_SPINNY_MOTOR = 23; // Update for proper ID - That is the correct ID
    public static final int SHOOTER_SUCK_MOTOR = 13; //Shooter suck motor
    public static final int SHOOTER_HOOD_PITCH = 14;
    public static final int SHOOTER_LAUNCH_MOTOR = 0;

    //Shooter Function Constants
    public static final double TURRET_ADJUST_SPEED = 0.2;
    public static final double MINIMUM_TURRET_ADJUST_SPEED = 0.075; // change through testing
    public static final double MANUAL_TURRET_SPEED = 0.2;
    public static final double SHOOTER_SUCK_SPEED = 0.7;
    public static final double SHOOTER_LAUNCH_SPEED_TARMAC = 0.565;//0.565
    public static final double SHOOTER_LAUNCH_SPEED_HUB = 0.4;//0.4
    public static final double SHOOTER_LAUNCH_SPEED_DISTANCE = 0.7;//0.7
    public static final double SHOOTER_LAUNCH_IDLE_SPEED = 0.2;
    public static final double SHOOTER_HOOD_UP_SPEED = 0.45;
    public static final double SHOOTER_HOOD_DOWN_SPEED = 0.15;
    public static final double MANUAL_SHOOTER_HOOD_UP = 0.1;
    public static final double MANUAL_SHOOTER_HOOD_DOWN = 0.05;
    public static final double SHOOTER_IDLE_SPEED = 0.55;
    public static final double TALON_COUNTSPERREV = 4096;
    public static final double TURN_TURRET_KP = -0.04; // Adjust as necessary
    public static final double HOOD_MOE = 0.7; // change as necessary;
    public static final double HOOD_KP = -0.05;

    //Rosbots Constants
    public static final double SHOOTER_LAUNCH_KP = 0.13744;
    public static final double SHOOTER_LAUNCH_KI = 0.0;
    public static final double SHOOTER_LAUNCH_KD = 0.0;
    public static final double SHOOTER_LAUNCH_TOLERANCE = 1.0;
    public static final double kSVolts = 0.63035;
    public static final double kVVoltSecondsPerRotation = 0.10877;
    public static final int kUnitsPerRevolution = 42;

    //Elevator Motors
    public static final int ELEVATOR_MOTOR_PULL_R = 22;
    public static final int ELEVATOR_MOTOR_PULL_L = 25;
    public static final double ELEVATOR_PULL_SPEED = 1.0;

    //Limelight Constants
    public static final double LIMELIGHT_MOUNTING_ANGLE_DEGREES = 70.0; //check?
    public static final double LIMELIGHT_LENS_HEIGHT = 36.5; //check?
    public static final double TURRET_SPINNY_ERROR_MARGIN = 3.0; // margin of error for the limelight when tracking hoop
    public static final double TURRETXP = 0.6;
    public static final double TURRETXI = 0;
    public static final double TURRETXD = 0;
    public static final double TURRET_TOLERANCE = 0.02;
    public static final double KP = -0.005; //Proportional Control Constant

    //Auto Constant1
    public static final double AUTONOMOUS_SPEED = 0.5;
    public static final double FALCON_COUNTSPERREV = 2048;
    public static final double DRIVE_GEARRATIO = 60/14;
    public static final double DRIVE_WHEELRADIUS = 6;
    public static final double AUTONOMOUS_TARGET_DISTANCE = 3.4;
    public static final double AUTO_INTAKE_TIME = 15.0;
    public static final double MIN_AUTO_ROTATION_SPEED = 0.1;
    public static final double AUTO_TURNING_KP = 0.0025; // Change as needed
    public static final double MIN_AUTO_DRIVE_SPEED = 0.25;
    public static final double AUTO_DISTANCE_KP = 0.0004; // Change as needed

    //Controller Constants
    public static final int JOYSTICK_NUMBER = 0;
    public static final int SHOOTER_JOYSTICK_NUMBER = 1;
    public static final int BUT_A = 1;
    public static final int BUT_B = 2;
    public static final int BUT_X = 3;
    public static final int BUT_Y = 4;
    public static final int BUT_LB = 5;
    public static final int BUT_RB = 6;
    public static final int BUT_M1 = 7;
    public static final int BUT_M2 = 8;
    public static final int JOY_POV = 0;
    public static final int LEFT_TRIG = 4;
    public static final int RIGHT_TRIG = 5;
    public static final int LEFT_JOY_X = 0;
    public static final int LEFT_JOY_Y = 1;
    public static final int RIGHT_JOY_X = 4;
    public static final int RIGHT_JOY_Y = 3;
}
