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
    //DriveBase
    public static final int LEFT_FRONT = 3;
    public static final int LEFT_BACK = 1;
    public static final int LEFT_TOP = 4;
    public static final int RIGHT_FRONT = 5;
    public static final int RIGHT_BACK = 2; 
    public static final int RIGHT_TOP = 6;

    //BONUS MOTORS
    public static final int INTAKE = 0;
    public static final int UPTAKE = 0;
    public static final int LEFT_BALLSUCC = 0;
    public static final int RIGHT_BALLSUCC = 0;
    public static final int GLOCK = 0;

    //SPEEDS
    public static final double DRIVE_SPEED = 0.7;
    public static final double INTAKE_SPEED = 0.3;
    public static final double UPTAKE_SPEED = 0.3;
    public static final double BALLSUCC = 0.6;
    public static final double GLOCK_SPEED = 0.5;

    //Autonomous
    public static final double DRIVE_FORWARD_TIME = 3;
    public static final double AUTOMOUS_SPEED = 0.5;
    
    //GLOCK
    public static final int SHOOTER_MOTOR = 5;

    //Intake
    public static final int INTAKE_MOTOR = 6;
    public static final int LEFTPISTON_A = 1;
    public static final int LEFTPISTON_B = 0;

    //Wheel Spinner
    public static final int WHEEL_MOTOR = 7;

    //Range Button Configurations
    public static final int CONTROLLER_NUMBER = 0;
    public static final int LEFT_JOY_X = 0;
    public static final int LEFT_JOY_Y = 1;
    public static final int LEFT_TRIG = 2;
    public static final int RIGHT_TRIG = 3;
    public static final int RIGHT_JOY_X = 4;
    public static final int RIGHT_JOY_Y = 5;

    //Binary Button Configurations
    public static final int BUT_A = 1;
    public static final int BUT_B = 2;
    public static final int BUT_X = 3;
    public static final int BUT_Y = 4;
    public static final int BUT_LB = 5;
    public static final int BUT_RB= 6;

    //-----------------------------PID VALUE-----------------------------//
    //DO NOT RUN ANYTHING OFF OF THESE VALUES
    public static final double shooterkP = .1;
    public static final double shooterkI = .01;
    public static final double shooterkD = .001;


	public static final int XBOX_PORT_ID = 0;
    public static final int GEAR_SHIFTER_FORWARD_CHANNEL = 0;
    public static final int GEAR_SHIFTER_REVERSE_CHANNEL = 0;
	
}
