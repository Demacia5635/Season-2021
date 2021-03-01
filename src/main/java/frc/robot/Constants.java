/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final int shooterWheelPort = -1;
    public static final int hoodMotorPort = -1;
    public static final int bonkerPort = -1;
    public static final int pulsePerRotation = 800;
    public static final int shooterKS = -1;
    public static final int vacuumMotor = -1;
    public static final int maxLim = -1;
    public static final int minLim = -1;

    public static int RIGHT_FRONT = 1;
    public static int RIGHT_BACK = 2;
    public static int LEFT_FRONT = 3;
    public static int LEFT_BACK = 4;
    public static int GYRO_PORT = 0;

    public static int XBOX_PORT = 0;

    public static final double ROBOT_TRACK_WIDTH = 0.575; // In meters
    public static double MAX_VELOCITY = 2; // In meters per second
    public static double MAX_RADIAL_ACCELARATION = 4; // In meters per squared second
    public static double MAX_ANGULAR_VELOCITY = Math.PI; // In radians per second

    public static double PULSES_PER_METER = 1675;

        // TO DO: SET!!!
        public static final int MOTION_S_CURVE = 3; // an integer between 0 - 8 that represents the
        // velocity smoothing
public static final double ACCELERATION = 10; // the acceletarion in sensor units per 100 ms
public static final double CRUISE_VELOCITY = 5 * PULSES_PER_METER / 10; // the peak speed in sensor
                                   // units per 100 ms

    public static final double CHASSIS_KP = 0.01;
    public static final double CHASSIS_KI = 0;
    public static final double CHASSIS_KD = 0;
    public static final double CHASSIS_KS = 0.797;
    public static final double CHASSIS_KV = 2.54;
    public static final double CHASSIS_KA = 0.862;
}
