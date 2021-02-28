/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX; // import the tlaonFX
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.SpeedControllerGroup; // import the speed control group type
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive; // import the diffrential drive
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
// some debugging power
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase; // import the base subsystem (which we extend)
import frc.robot.Constants; // import all the measured constants
import frc.robot.commands.Drive.DriveStates;
import frc.robot.utils.GroupOfMotors;

public class Chassis extends SubsystemBase implements Sendable {

  // TO DO: check the engines direction, maybe invert
  private WPI_TalonSRX rightFront;
  private WPI_TalonSRX leftFront;
  private WPI_TalonSRX rightBack; 
  private WPI_TalonSRX leftBack;
  private final DifferentialDriveOdometry m_odometry;
  private GroupOfMotors right;
  private GroupOfMotors left;
  private PigeonIMU gyro;
  private DifferentialDrive m_drive; // instance of the premade diffrential drive
  private SpeedControllerGroup leftMotors; // a group which contains both left motors
  private SpeedControllerGroup rightMotors; // a group which contains both right motors
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.ks,
      Constants.kv, Constants.ka);

  /**
   * Creates a new Chassis.
   */
  public Chassis(DriveStates dStates) {
    rightFront = new WPI_TalonSRX(Constants.rightFront);
    leftFront = new WPI_TalonSRX(Constants.leftFront);
    rightBack = new WPI_TalonSRX(Constants.rightBack);
    leftBack = new WPI_TalonSRX(Constants.leftBack);

    rightFront.setInverted(true);
    leftFront.setInverted(true);
    rightBack.setInverted(true);
    leftBack.setInverted(true);

    if (dStates == DriveStates.arcadeDrive || dStates == DriveStates.curvatureDrive) {
      this.leftMotors = new SpeedControllerGroup(leftFront, leftBack);
      this.rightMotors = new SpeedControllerGroup(rightBack, rightBack);
      this.m_drive = new DifferentialDrive(this.leftMotors, this.rightMotors);
      this.m_drive.setMaxOutput(0.5);
      m_drive.setRightSideInverted(false);
    } else {
      this.right = new GroupOfMotors(rightFront, rightBack);
      this.left = new GroupOfMotors(leftFront, leftBack);
      this.gyro = new PigeonIMU(Constants.gyroPort);
      this.gyro.setFusedHeading(0);
    }
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(gyro.getFusedHeading()));
  }

  public void setVelocity(double left, double right) {
    this.left.setVelocity(left, feedforward);
    this.right.setVelocity(right, feedforward);
  }

  public double getAngle() {
    double angle = gyro.getFusedHeading();

    if (angle < 0) {
      angle = -((-angle) % 360.0);
      if (angle < -180) {
        return 360.0 + angle;
      }
      return angle;
    }
    angle = angle % 360.0;
    if (angle > 180) {
      return angle - 360.0;
    }
    return angle;
  }

  public double getRightPos() {
    return right.getDistance();
  }

  public double getLeftPos() {
    return left.getDistance();
  }

  public double getRightVelocity() {
    return right.getVelocity();
  }

  public double getLeftVelocity() {
    return left.getVelocity();
  }

  /** 
  gets 2 values between 1 to -1 one to determine the tangent velocity 
  and the other determines the radial accelaration of the robot

  the function sets calculated values for the right and left motors
  */
  public void radialAccelaration(double velocity, double turns) {
    velocity = velocity * Constants.maxVelocity;
    turns = turns * Constants.maxRadialAccelaration;
    double right = 0;
    double left = 0;
    if (velocity != 0) {
      if (turns > 0) {
        double radius = (velocity * velocity/turns);
        right = (velocity/radius)*(radius-(Constants.robotLength/2));
        left = (velocity/radius)*(radius+(Constants.robotLength/2));
      } 
      else if (turns < 0) {
        double radius = (velocity * velocity/(-turns));
        right = (velocity/radius)*(radius+(Constants.robotLength/2));
        left = (velocity/radius)*(radius-(Constants.robotLength/2));
      } 
      else {
        right = velocity;
        left = velocity;
      }
    } 
    else {
      if (turns > 0) {
        right = -Math.sqrt(turns * (Constants.robotLength / 2));
        left = Math.sqrt(turns * (Constants.robotLength / 2));
      } 
      else {
        right = Math.sqrt((-turns) * (Constants.robotLength / 2));
        left = -Math.sqrt((-turns) * (Constants.robotLength / 2));
      }
    }
    setVelocity(left, right);
  }

  /** 
  gets 2 values between 1 to -1 one to determine the tangent velocity 
  and the other determines the angular velocity of the robot
   
  the function sets calculated values for the right and left motors
  */
  public void angularVelocity(double velocity, double turns) {
    velocity = velocity * Constants.maxVelocity;
    turns = turns * Constants.maxAngularVelocity;
    double right = 0;
    double left = 0;
    if (velocity > 0) {
      right = velocity - turns * (Constants.robotLength / 2);
      left = velocity + turns * (Constants.robotLength / 2);
    } 
    else if (velocity < 0) {
      right = velocity + turns * (Constants.robotLength / 2);
      left = velocity - turns * (Constants.robotLength / 2);
    } 
    else {
      right = -turns * (Constants.robotLength / 2);
      left = turns * (Constants.robotLength / 2);
    }
    setVelocity(left, right);
  }

  public void arcadeDrive(double xSpeed, double zRotation, boolean squareInputs) {
    // xSpeed - The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
    // zRotation - The robot's rotation rate around the Z axis [-1.0..1.0].
    // Clockwise is positive.
    // squareInputs - If set, decreases the input sensitivity at low speeds.

    this.m_drive.arcadeDrive(xSpeed, zRotation, squareInputs);
  }

  public void curvatureDrive(double xSpeed, double zRotation, boolean isQuickTurn) {
    // xSpeed - The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
    // zRotation - The robot's rotation rate around the Z axis [-1.0..1.0].
    // Clockwise is positive.
    // isQuickTurn - If set, overrides constant-curvature turning for turn-in-place
    // maneuvers.

    this.m_drive.curvatureDrive(xSpeed, zRotation, isQuickTurn);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    // builder.addDoubleProperty(key, getter, setter);
    builder.addDoubleProperty("Left Speed", this::getLeftVelocity, null);
    builder.addDoubleProperty("Right Speed", this::getRightVelocity, null);
    builder.addDoubleProperty("Angle", this::getAngle, null);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
  *
  * @param pose The pose to which to set the odometry.
  */
  public void resetOdometry(Pose2d pose) {
    leftFront.setSelectedSensorPosition(0);
    rightFront.setSelectedSensorPosition(0);
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(gyro.getFusedHeading()));
  }

  public double get_ks() {
    return Constants.ks;
  }

  public double get_kv() {
    return Constants.kv;
  }

  public double get_kp() {
    return Constants.kp;
  }

  public double SpeedInMtoSec1() {
    return this.getLeftVelocity() * 10 / Constants.pulsesPerMeter;
  }

  public double SpeedInMtoSec2() {
    return this.getRightVelocity() * 10 / Constants.pulsesPerMeter;
  }
}
