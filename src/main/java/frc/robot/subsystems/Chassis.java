/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX; // import the tlaonFX
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.SpeedControllerGroup; // import the speed control group type
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive; // import the diffrential drive
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
// some debugging power
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase; // import the base subsystem (which we extend)
import frc.robot.Constants; // import all the measured constants
import frc.robot.commands.Drive.DriveStates;
import frc.robot.utils.GroupOfMotors;

public class Chassis extends SubsystemBase {

  // TO DO: check the engines direction, maybe invert
  private final DifferentialDriveOdometry m_odometry;
  private GroupOfMotors right;
  private GroupOfMotors left;
  private PigeonIMU gyro;
  private DifferentialDrive m_drive; // instance of the premade diffrential drive
  private SpeedControllerGroup leftMotors; // a group which contains both left motors
  private SpeedControllerGroup rightMotors; // a group which contains both right motors
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
      Constants.CHASSIS_KS, Constants.CHASSIS_KV, Constants.CHASSIS_KA);

  /**
   * Creates a new Chassis.
   */
  public Chassis(DriveStates dStates) {
    WPI_TalonSRX rightFront = new WPI_TalonSRX(Constants.RIGHT_FRONT);
    WPI_TalonSRX leftFront = new WPI_TalonSRX(Constants.LEFT_FRONT);
    WPI_TalonSRX rightBack = new WPI_TalonSRX(Constants.RIGHT_BACK);
    WPI_TalonSRX leftBack = new WPI_TalonSRX(Constants.LEFT_BACK);

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
      this.gyro = new PigeonIMU(Constants.GYRO_PORT);
      this.gyro.setFusedHeading(0);
    }
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(gyro.getFusedHeading()));
  }

  public void setVelocity(double left, double right) {
    this.left.setVelocity(left, feedforward);
    this.right.setVelocity(right, feedforward);
  }

  public double getFusedHeading(){
    return gyro.getFusedHeading();
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
   * gets 2 values between 1 to -1 one to determine the tangent velocity
   * and the other determines the radial accelaration of the robot
   * 
   * the function sets calculated values for the right and left motors
   */
  public void radialAccelaration(double velocity, double turns) {
    velocity = velocity * Constants.MAX_VELOCITY;
    turns = turns * Constants.MAX_RADIAL_ACCELARATION;
    double right = 0;
    double left = 0;
    if (velocity != 0) {
      if (turns > 0) {
        double radius = (velocity * velocity / turns);
        right = (velocity / radius) * (radius - (Constants.ROBOT_TRACK_WIDTH / 2));
        left = (velocity / radius) * (radius + (Constants.ROBOT_TRACK_WIDTH / 2));
      } else if (turns < 0) {
        double radius = (velocity * velocity / (-turns));
        right = (velocity / radius) * (radius + (Constants.ROBOT_TRACK_WIDTH / 2));
        left = (velocity / radius) * (radius - (Constants.ROBOT_TRACK_WIDTH / 2));
      } else {
        right = velocity;
        left = velocity;
      }
    } else {
      if (turns > 0) {
        right = -Math.sqrt(turns * (Constants.ROBOT_TRACK_WIDTH / 2));
        left = Math.sqrt(turns * (Constants.ROBOT_TRACK_WIDTH / 2));
      } else {
        right = Math.sqrt((-turns) * (Constants.ROBOT_TRACK_WIDTH / 2));
        left = -Math.sqrt((-turns) * (Constants.ROBOT_TRACK_WIDTH / 2));
      }
    }
    setVelocity(left, right);
  }

  /**
   * gets 2 values between 1 to -1 one to determine the tangent velocity
   * and the other determines the angular velocity of the robot
   * 
   * the function sets calculated values for the right and left motors
   */
  public void angularVelocity(double velocity, double turns) {
    velocity = velocity * Constants.MAX_VELOCITY;
    turns = turns * Constants.MAX_ANGULAR_VELOCITY;
    double right = 0;
    double left = 0;
    if (velocity > 0) {
      right = velocity - turns * (Constants.ROBOT_TRACK_WIDTH / 2);
      left = velocity + turns * (Constants.ROBOT_TRACK_WIDTH / 2);
    } else if (velocity < 0) {
      right = velocity + turns * (Constants.ROBOT_TRACK_WIDTH / 2);
      left = velocity - turns * (Constants.ROBOT_TRACK_WIDTH / 2);
    } else {
      right = -turns * (Constants.ROBOT_TRACK_WIDTH / 2);
      left = turns * (Constants.ROBOT_TRACK_WIDTH / 2);
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
    this.left.resetEncoder();
    this.right.resetEncoder();
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(gyro.getFusedHeading()));
  }

  public double get_ks() {
    return Constants.CHASSIS_KS;
  }

  public double get_kv() {
    return Constants.CHASSIS_KV;
  }

  public double get_kp() {
    return Constants.CHASSIS_KP;
  }

  public double SpeedInMtoSec1() {
    return this.getLeftVelocity() * 10 / Constants.PULSES_PER_METER;
  }

  public double SpeedInMtoSec2() {
    return this.getRightVelocity() * 10 / Constants.PULSES_PER_METER;
  }

  /**
   * Drives to the ball on an arc
   * 
   * @param speed - The velocity at which the robot will drive in Meters
   */
  public void driveToBall(double speed) {
    double distance = SmartDashboard.getNumber("VisionDistance", 0);
    double angle = SmartDashboard.getNumber("VisionAngle", 0);
    double radius = distance / (2 * Math.sin(angle * Math.PI / 180));
    double k = Constants.ROBOT_TRACK_WIDTH * 100 / 2;
    double left = speed * (1 + (k / radius));
    double right = speed * (1 - (k / radius));
    setVelocity(left * Constants.MAX_VELOCITY, right * Constants.MAX_VELOCITY);
  }

  /**
   * 
   * @param speed - The velocity at which the robot will drive in Meters
   * 
   * @return A command that will execute the driveToBall function pereiodecly
   *         untill reacing the ball
   */
  public CommandBase driveToBallCommand(double speed) {
    return new FunctionalCommand(() -> {
    }, () -> {
      driveToBall(speed);
    }, (interrupted) -> {
      setVelocity(0, 0);
    }, () -> {
      return SmartDashboard.getNumber("VisionDistance", 0) == 0;
    });
  }

  public void setPos(double pos1, double pos2) {
    this.left.setMotionMagic(pos1, this.feedforward, Constants.CRUISE_VELOCITY,
        Constants.ACCELERATION);
    this.right.setMotionMagic(pos1, this.feedforward, Constants.CRUISE_VELOCITY,
        Constants.ACCELERATION);
  }

  public void setPos2(double pos1, double pos2) {
    this.left.setMotionMagic(pos1);
    this.right.setMotionMagic(pos1);
  }

  public void goTo(double distanceLeft, double distanceRight) {
    // this.configMotionMagic();
    this.setPos2(this.left.getEncoder() + distanceLeft, this.right.getEncoder() + distanceRight);
  }

  public void goTo(double distance) {
    // this.configMotionMagic();
    this.setPos2(this.left.getEncoder() + distance, this.right.getEncoder() + distance);
  }

  public void configMotionMagic() {
    this.left.setMotionSCurve(Constants.MOTION_S_CURVE);
    this.left.setCruiseVelocity(Constants.CRUISE_VELOCITY);
    this.left.setAcceleration(Constants.ACCELERATION);
    this.right.setMotionSCurve(Constants.MOTION_S_CURVE);
    this.right.setCruiseVelocity(Constants.CRUISE_VELOCITY);
    this.right.setAcceleration(Constants.ACCELERATION);
  }

  public void configMotionMagic(double accelaration) {
    this.left.setMotionSCurve(Constants.MOTION_S_CURVE);
    this.left.setCruiseVelocity(Constants.CRUISE_VELOCITY);
    this.left.setAcceleration(accelaration);
    this.right.setMotionSCurve(Constants.MOTION_S_CURVE);
    this.right.setCruiseVelocity(Constants.CRUISE_VELOCITY);
    this.right.setAcceleration(accelaration);
  }

  public void configMotionMagic(int curve) {
    this.left.setMotionSCurve(curve);
    this.left.setCruiseVelocity(Constants.CRUISE_VELOCITY);
    this.left.setAcceleration(Constants.ACCELERATION);
    this.right.setMotionSCurve(curve);
    this.right.setCruiseVelocity(Constants.CRUISE_VELOCITY);
    this.right.setAcceleration(Constants.ACCELERATION);
  }

  public void configMotionMagic(double accelaration, int curve) {
    this.left.setMotionSCurve(curve);
    this.left.setCruiseVelocity(Constants.CRUISE_VELOCITY);
    this.left.setAcceleration(accelaration);
    this.right.setMotionSCurve(curve);
    this.right.setCruiseVelocity(Constants.CRUISE_VELOCITY);
    this.right.setAcceleration(accelaration);
  }

  public double getPosLeft() {
    return this.left.getEncoder();
  }

  public double getPoseRight() {
    return this.right.getEncoder();
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
}
