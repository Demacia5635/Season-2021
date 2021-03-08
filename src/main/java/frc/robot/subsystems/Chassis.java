/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX; // import the tlaonFX
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.SpeedControllerGroup; // import the speed control group type
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive; // import the diffrential drive
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
// some debugging power
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase; // import the base subsystem (which we extend)
import frc.robot.Constants; // import all the measured constants
import frc.robot.RobotContainer;
import frc.robot.commands.Drive.DriveStates;
import frc.robot.utils.GroupOfMotors;

public class Chassis extends SubsystemBase {

  // TO DO: check the engines direction, maybe invert
  private final DifferentialDriveOdometry m_odometry;
  private GroupOfMotors right;
  private GroupOfMotors left;
  private PigeonIMU gyro;
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
      Constants.CHASSIS_KS, Constants.CHASSIS_KV, Constants.CHASSIS_KA);
  private final Field2d field = new Field2d();

  /**
   * Creates a new Chassis.
   */
  public Chassis() {

    WPI_TalonFX rightFront = new WPI_TalonFX(Constants.RIGHT_FRONT);
    WPI_TalonFX leftFront = new WPI_TalonFX(Constants.LEFT_FRONT);
    WPI_TalonFX rightBack = new WPI_TalonFX(Constants.RIGHT_BACK);
    WPI_TalonFX leftBack = new WPI_TalonFX(Constants.LEFT_BACK);

    rightFront.setInverted(true);
    rightBack.setInverted(true);

    this.right = new GroupOfMotors(rightFront, rightBack);
    this.left = new GroupOfMotors(leftFront, leftBack);
    this.gyro = RobotContainer.gyro;
    left.resetEncoder();
    right.resetEncoder();
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getFusedHeading()));

    SmartDashboard.putData("Field", field);
  }

  public void setVelocity(double left, double right) {
    this.left.setVelocity(left, feedforward);
    this.right.setVelocity(right, feedforward);
  }

  public double getFusedHeading() {
    if (gyro != null) {
      return gyro.getFusedHeading();
    } else {
      gyro = RobotContainer.gyro;
      if (gyro != null) {
        return gyro.getFusedHeading();
      }
    }
    return 0;
  }

  public double getAngle() {
    double angle = getFusedHeading();

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

  /**
   * Finds the nearest ball and drives to it.
   * 
   * @param isClockwise defines whether the robot would look for the ball by
   *                    turning clockwise or counterclockwise.
   * 
   * @return the command that finds, and drives to the ball
   */
  public Command findAndDriveToBall(boolean isClockwise) {
    return SequentialCommandGroup.sequence(new RunCommand(() -> {
      setVelocity((isClockwise ? 1 : -1) * Constants.MAX_AUTOMATION_VELOCITY / 2,
          (isClockwise ? -1 : 1) * Constants.MAX_AUTOMATION_VELOCITY / 2);
    }, this).withInterrupt(() -> (SmartDashboard.getNumber("VisionAngle", 0) == 0)),
        driveToBallCommand(Constants.MAX_AUTOMATION_VELOCITY));
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

  public void configMotionMagic(double accelaration, double curve) {
    for (GroupOfMotors motor : new GroupOfMotors[] { left, right }) {
      motor.setMotionSCurve((int) curve);
      motor.setCruiseVelocity(Constants.CRUISE_VELOCITY);
      motor.setAcceleration(accelaration);
    }
  }

  public void configMotionMagic() {
    configMotionMagic(Constants.ACCELERATION, Constants.MOTION_S_CURVE);
  }

  public void configMotionMagic(double acceleration) {
    configMotionMagic(acceleration, Constants.MOTION_S_CURVE);
  }

  public void configMotionMagic(int curve) {
    configMotionMagic(Constants.ACCELERATION, curve);
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
    m_odometry.update(getRotation2d(), left.getDistance(), right.getDistance());
    field.setRobotPose(m_odometry.getPoseMeters());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    // builder.addDoubleProperty(key, getter, setter);
    builder.addDoubleProperty("Left Distance", this::getLeftPos, null);
    builder.addDoubleProperty("Right Distance", this::getRightPos, null);
    builder.addDoubleProperty("Left Speed", this::getLeftVelocity, null);
    builder.addDoubleProperty("Right Speed", this::getRightVelocity, null);
    builder.addDoubleProperty("Angle", this::getAngle, null);
  }

  // public void setPower(int left, int right) {
  // this.left.setPower(left);
  // this.right.setPower(right);
  // }
  public void setPower(double left, double right) {
    this.left.setPower(left);
    this.right.setPower(right);
  }

  public double getRightDistance() {
    return this.right.getDistance();
  }

  public double getLeftDistance() {
    return this.left.getDistance();
  }

  public double getChassisDistance() {
    return (this.getLeftDistance() + this.getRightDistance()) / 2;
  }

  /**
   * 
   * @param angle - an angle between 0 to 360
   * 
   * @return - return the angle between 180 to -180
   */
  public double normalizeAngle(double angle) {
    return Math.IEEEremainder(angle, 360);
  }

  public double getNormalizedAngle() { // returns the angle of the robot between 180 to -180
    return normalizeAngle(getAngle());
  }

  /**
   * 
   * @param reqAngle
   * @param curAngle
   * 
   * @return returns the smallest delta between the angles
   */
  public double diffAngle(double reqAngle, double curAngle) { // returns the shortest angle to what
                                                              // you want
    double a1 = normalizeAngle(reqAngle) - normalizeAngle(curAngle);
    if (a1 <= -180) {
      return a1 + 360;
    } else if (a1 > 180) {
      return a1 - 360;
    } else {
      return a1;
    }
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getAngle());
  }

  public double getAngle2Pose(Pose2d pose) {
    Translation2d translation2d = pose.getTranslation().minus(getPose().getTranslation());
    return new Rotation2d(translation2d.getX(), translation2d.getY()).getDegrees();
  }

}
