// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

    //Indexing Motor IDs
    public static final int INDEX_LEFT = 11; //Indexing Left
    public static final int INDEX_RIGHT = 12; //Indexing Right
    public static final double INDEX_SPEED = -1;

    public static final int INTAKE_MOTOR = 24;
    public static final int INTAKE_MOVE_MOTOR = 21;
    public static final double INTAKE_SPEED = -1;
    public static final double INTAKE_MOVE_SPEED_DOWN = .3;
    public static final double INTAKE_MOVE_SPEED_UP = .65;

    public static final double INTAKE_MOVEUP_TIME1 = 1.0;
    public static final double INTAKE_MOVEUP_TIME2 = .2;
    public static final double INTAKE_MOVEDOWN_TIME1 = 0.2;
    public static final double INTAKE_MOVEDOWN_TIME2 = 0.5;

    //Shooter Motor ID's
    public static final int TURRET_SPINNY_MOTOR = 23; // Update for proper ID - That is the correct ID
    public static final double TURRET_ADJUST_SPEED = 0.3;
    public static final double MINIMUM_TURRET_ADJUST_SPEED = 0.3; // change through testing
    public static final double MANUAL_TURRET_SPEED = 0.3;
    public static final int SHOOTER_SUCK_MOTOR = 13; //Shooter suck motor
    public static final int SHOOTER_HOOD_PITCH = 14;
    public static final int SHOOTER_LAUNCH_MOTOR = 0;

    public static final double SHOOTER_SUCK_SPEED = 0.7;
    public static final double SHOOTER_LAUNCH_SPEED = 0.4;
    public static final double SHOOTER_HOOD_UP_SPEED = 0.45;
    public static final double SHOOTER_HOOD_DOWN_SPEED = 0.15;
    public static final double MANUAL_SHOOTER_HOOD_UP = 0.25;
    public static final double MANUAL_SHOOTER_HOOD_DOWN = 0.10;
    public static final double SHOOTER_IDLE_SPEED = 0.55;
    public static final double TALON_COUNTSPERREV = 4096;
    public static final double TURN_TURRET_KP = 0; // Adjust as necessary

    public static final double HOOD_MOE = 0.7; // change as necessary;
    public static final double HOOD_KP = -0.05;
    public static final double HOOD_PITCH_KP = 0;
    public static final double HOOD_PITCH_KI = 0;
    public static final double HOOD_PITCH_KD = 0;



    //Rosbots Constants
    public static final double SHOOTER_LAUNCH_KP = 0.13744;
    public static final double SHOOTER_LAUNCH_KI = 0.0;
    public static final double SHOOTER_LAUNCH_KD = 0.0;
    public static final double SHOOTER_LAUNCH_TOLERANCE = 1.0;
    public static final double kSVolts = 0.63035;
    public static final double kVVoltSecondsPerRotation = 0.10877;
    public static final int kUnitsPerRevolution = 42;

    //Elevator Motors
    public static final int ELEVATOR_MOTOR_PULL_R = 22; //Should be good just double check
    public static final int ELEVATOR_MOTOR_PULL_L = 25; //Should be good just double check
    public static final int ELEVATOR_TILT_R = 30;
    public static final int ELEVATOR_TILT_L = 31;
    public static final double ELEVATOR_PULL_SPEED = 0.5;
    public static final double ELEVATOR_TILT_SPEED = 0.2;

    //Limelight Constants
    public static final double LIMELIGHT_MOUNTING_ANGLE_DEGREES = 25; //check?
    public static final double LIMELIGHT_LENS_HEIGHT = 37; //check?
    public static final double TURRET_SPINNY_ERROR_MARGIN = 3.0; // margin of error for the limelight when tracking hoop
    public static final double TURRETXP = 0.6;
    public static final double TURRETXI = 0;
    public static final double TURRETXD = 0;

    //Other Constants
    public static final double DRIVETRAINSPEED = 0.7;
    public static final double DRIVE_FORWARD_TIME = 3.0;
    public static final int JOYSTICK_NUMBER = 0;
    public static final int SHOOTER_JOYSTICK_NUMBER = 1;
    public static final double AUTONOMOUS_TARGET_DISTANCE = 3.4;
    public static final double AUTO_INTAKE_TIME = 15.0;
    public static final double KP = -0.005; //Proportional Control Constant
    public static final double TURRET_TOLERANCE = 0.05;

    //Auto Constants
    public static final double AUTONOMOUS_SPEED = 0.6;
    public static final double FALCON_COUNTSPERREV = 2048;
    public static final double DRIVE_GEARRATIO = 60/14;
    public static final double DRIVE_WHEELRADIUS = 6;

    //Binary Button Configurations
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
