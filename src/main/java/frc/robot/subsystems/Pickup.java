/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Pickup extends SubsystemBase {

  private WPI_TalonSRX pickupMotor;
  private WPI_TalonSRX armMotor;
  private PigeonIMU gyro;

  public Pickup() {
    pickupMotor = new WPI_TalonSRX(Constants.PICKUP_MOTOR_PORT);
    armMotor = new WPI_TalonSRX(Constants.ARM_MOTOR_PORT);
    armMotor.config_kP(0, Constants.ARM_KP);
    armMotor.setInverted(true);
    armMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
        LimitSwitchNormal.NormallyOpen);
    gyro = new PigeonIMU(pickupMotor);
    gyro.setFusedHeading(0);
    RobotContainer.gyro = gyro;
    pickupMotor.configForwardSoftLimitThreshold(Constants.MAX_ARM_POS);
    pickupMotor.configForwardSoftLimitEnable(true);
    //setDefaultCommand(getarmMoveCommand());
  }

  /**
   * Starts the pickup.
   */
  public void startPickup() {
    pickupMotor.set(ControlMode.PercentOutput, 1);
  }

  /**
   * Stops the pickup.
   */
  public void stopPickup() {
    pickupMotor.set(ControlMode.PercentOutput, 0);
  }

  /**
   * 
   * @return The arm motor angle in degrees : 0 = start position.
   */
  public double getArmPosition() {
    return armMotor.getSelectedSensorPosition();
  }

  /**
   * Sets the angle of the arm motor.
   * 
   * @param power between 1 and -1.
   */
  public void setArm(double power) {
    armMotor.set(ControlMode.PercentOutput, power);
  }

  /**
   * 
   * @return A StartEndCommand that starts the pickup.
   */
  public StartEndCommand getPickupCommand() {
    return new StartEndCommand(this::startPickup, this::stopPickup, this);
  }

  public double getArmLimit(){
    return armMotor.isRevLimitSwitchClosed();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (getArmLimit() == 1){
      armMotor.setSelectedSensorPosition(0);
    }
  }

  public void moveArm() {
    armMotor.set(ControlMode.PercentOutput, 0.25);
  }

  public void stopArm(boolean a){
    armMotor.set(ControlMode.PercentOutput, 0);
  }

  private boolean rFalse(){
    return false;
  }

  public Command getarmMoveCommand(){
    return new FunctionalCommand(this::moveArm, this::moveArm,this::stopArm,this::rFalse , this);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Arm Position", this::getArmPosition, null);
    builder.addDoubleProperty("Arm Limit", this::getArmLimit, null);
    armMotor.initSendable(builder);
  }
}
