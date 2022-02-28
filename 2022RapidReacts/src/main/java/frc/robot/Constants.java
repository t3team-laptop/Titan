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
    public static final int INDEX_TOP = 13; //Indexing Top
    public static final double INDEX_SPEED = -1;

    public static final int INTAKE_MOTOR = 24;
    public static final int INTAKE_MOVE_MOTOR = 22;
    public static final double INTAKE_SPEED = 1;
    public static final double INTAKE_MOVE_SPEED_DOWN = .3;
    public static final double INTAKE_MOVE_SPEED_UP = .65;

    //Other Constants
    public static final double DRIVETRAINSPEED = 0.7;
    public static final double DRIVE_FORWARD_TIME = 3.0;
    public static final double AUTONOMOUS_SPEED = 0.2;
    public static final int JOYSTICK_NUMBER = 0;    
    public static final double AUTONOMOUS_TARGET_DISTANCE = 3.4;

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
    public static final int LEFT_TRIG = 2;
    public static final int RIGHT_TRIG = 3;
}
