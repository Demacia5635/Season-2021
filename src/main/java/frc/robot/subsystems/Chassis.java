/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Chassis extends SubsystemBase {
  private final TalonSRX BackLeft = new TalonSRX(Constants.BackLeftID);
  private final TalonSRX BackRight = new TalonSRX(Constants.BackRightID);
  private final TalonSRX FrontLeft = new TalonSRX(Constants.FrontLeftID);
  private final TalonSRX FrontRight = new TalonSRX(Constants.FrontRightID);
  private final PigeonIMU gyro = new PigeonIMU(Constants.GyroID);
  private final DifferentialDriveOdometry odometry;
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.ks,
      Constants.kv, Constants.ka);

  private double baseAngle;

  /**
   * Creates a new Chassis.
   */
  public Chassis() {
    BackRight.follow(FrontRight);
    FrontLeft.follow(BackLeft);

    BackLeft.config_kP(0, Constants.kp);
    FrontLeft.config_kP(0, Constants.kp);
    FrontRight.config_kP(0, Constants.kp);
    BackRight.config_kP(0, Constants.kp);

    resetEncoders();
    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getAngle()));

    baseAngle = 0;
  }

  @Override
  public void periodic() {
    odometry.update(Rotation2d.fromDegrees(getAngle()), getLeftDistance(), getRightDistance());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public double getAngle() {
    return gyro.getFusedHeading() + baseAngle;
  }

  public double getLeftDistance() {
    return BackLeft.getSelectedSensorPosition() / Constants.pulseInMeter;
  }

  public double getRightDistance() {
    return FrontRight.getSelectedSensorPosition() / Constants.pulseInMeter;
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    resetGyro(pose.getRotation().getDegrees());
    odometry.resetPosition(pose, pose.getRotation());
  }

  public void resetGyro() {
    gyro.setFusedHeading(0);
  }

  public void resetGyro(double angle) {
    baseAngle = angle - getAngle();
  }

  public void resetEncoders() {
    BackLeft.setSelectedSensorPosition(0);
    FrontRight.setSelectedSensorPosition(0);
  }

  /**
   * Controls the left and right sides of the drive directly with velocities.
   *
   * @param leftVelocity  the commanded left velocity
   * @param rightVelocity the commanded right velocity
   */
  public void setVelocity(double leftVelocity, double rightVelocity) {
    setVelocity(leftVelocity, true);
    setVelocity(rightVelocity, false);
  }

  /**
   * Controls either one of the sides of the drive directly with velocity.
   *
   * @param vel    the wanted velocity
   * @param isLeft whether it should change the left or right side's velocities
   */
  public void setVelocity(double vel, boolean isLeft) {
    double velPulse = vel * Constants.pulseInMeter / 10;
    double a = velPulse - ((isLeft ? BackLeft : FrontRight).getSelectedSensorVelocity()
        / Constants.pulseInMeter * 10);
    double aFeedforward = feedforward.calculate(velPulse, a) / 12;

    (isLeft ? BackLeft : FrontRight).set(ControlMode.Velocity, velPulse,
        DemandType.ArbitraryFeedForward, aFeedforward);
  }
}
