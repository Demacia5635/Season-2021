/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    /**
     * Shooter
     */

    public static final int SHOOTER_WHEEL_PORT = -1;
    public static final int HOOD_MOTOR_PORT = -1;
    public static final int BONKER_PORT = -1;
    public static final int PULSE_PER_ROTATION = 800;
    public static final int SHOOTER_KS = -1;
    public static final int VACUM_MOTOR = -1;
    public static final double SHOOTER_DIAMETER = -1; // in meters

    /**
     * Path Following
     */

    // DifferentialDriveKinematics
    public static final double TRACKWIDTH_METERS = 0.585;
    public static final DifferentialDriveKinematics DRIVE_KINEMATICS =
                                                                     new DifferentialDriveKinematics(
                                                                             TRACKWIDTH_METERS);

    // Ramsete Parameters (in meters and seconds)
    public static final double RAMSETE_B = 2;
    public static final double RAMSETE_ZETA = 0.7;

    /**
     * Yarmus Constants
     */

    // Ports
    public static final int BackRightID = 1;
    public static final int BackLeftID = 4;
    public static final int FrontRightID = 2;
    public static final int FrontLeftID = 3;
    public static final int GyroID = 3;

    // Wheel shit
    public static final double CheesyPuffsMeter = 2.54 / 100.;
    public static final int wheelD = 6; /// in inches
    public static final double D = wheelD * CheesyPuffsMeter; // in meters

    // Pulses
    public static final double P = D * Math.PI; // wheel paremeter
    public static final double pulses = 800;
    public static final double pulsesToMeter = (1 / Constants.P) * Constants.pulses;
    public static final double pulseInMeter = P / pulses; 

    // Characteristics
    public static final double kp = 0.0001;
    public static final double kd = 0;
    public static final double ki = 0;
    public static final double ks = 0.797;
    public static final double kv = 2.34;
    public static final double ka = 0.862;
}
