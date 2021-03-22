/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be
 * declared globally (i.e. public static). Do not put anything functional in
 * this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final int SHOOTER_WHEEL_PORT = 10;
    public static final int HOOD_MOTOR_PORT = 8;
    public static final int BONKER_PORT = 7;
    public static final int VACUUM_MOTOR_PORT = 9;
    public static final int LIM_PORT = 0;
    public static final int ROULETTE_MOTOR_PORT = 11;
    public static final int ARM_MOTOR_PORT = 5;
    public static final int PICKUP_MOTOR_PORT = 6;
    public static final double ZERO_SHOOT_DISTANCE = 140;

    public static final double SHOOTER_KS = 1. / 10900.;
    public static final double SHOOTER_KV = 0.09;
    public static final double SHOOTER_KP = 0.05;
    public static final double HOOD_KP = 0.1;
    public static final double HOOD_KI = 0.01;
    public static final double ARM_KP = -1;

    public static final double ROULETTE_ROTATION_PER_SEC = 0.5;

    public static final int RIGHT_FRONT = 1;
    public static final int RIGHT_BACK = 2;
    public static final int LEFT_FRONT = 3;
    public static final int LEFT_BACK = 4;
    public static final int GYRO_PORT = 6;

    public static final int XBOX_PORT = 0;

    public static final double ROBOT_TRACK_WIDTH = 0.59; // In meters
    public static final double MAX_VELOCITY = 2.5; // In meters per second
    public static final double MAX_RADIAL_ACCELARATION = 4; // In meters per squared second
    public static final double MAX_ANGULAR_VELOCITY = Math.PI; // In radians per second

    public static final int MAX_ARM_POS = 150;

    public static final double PULSES_PER_METER = 44700;

    public static final int MOTION_S_CURVE = 3; // an integer between 0 - 8 that represents the
    // velocity smoothing
    public static final double ACCELERATION = 10; // the acceletarion in sensor units per 100 ms
    public static final double CRUISE_VELOCITY = 5 * PULSES_PER_METER / 10; // the peak speed in
    // sensor
    // units per 100 ms

    public static final double CHASSIS_KP = 0.0000145;
    public static final double CHASSIS_KI = 0;
    public static final double CHASSIS_KD = 0;
    public static final double CHASSIS_KS = 0.638;
    public static final double CHASSIS_KV = 2.28;
    public static final double CHASSIS_KA = 0.245;

    /**
     * Galactic Search
     */
    public static final double CHALLENGE_SPACE_WIDTH = 9.15;
    public static final double CHALLENGE_SPACE_HEIGHT = 4.58;
    public static final double MAX_AUTOMATION_VELOCITY = 4;
    public static final double MAX_AUTOMATION_ACCELERATION = 2;

    public static final double INCHES_TO_METERS = 0.0254;

    /**
     * AutoNav
     */

    // DifferentialDriveKinematics
    public static final DifferentialDriveKinematics DRIVE_KINEMATICS =
                                                                     new DifferentialDriveKinematics(
                                                                             ROBOT_TRACK_WIDTH);

    // Ramsete Parameters (in meters and seconds)
    public static final double RAMSETE_B = 2;
    public static final double RAMSETE_ZETA = 0.7;
}
