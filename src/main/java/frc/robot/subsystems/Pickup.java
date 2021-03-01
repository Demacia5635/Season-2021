/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pickup extends SubsystemBase {

  private WPI_TalonSRX pickupMotor;
  private WPI_TalonSRX armMotor;

  public Pickup() {
    pickupMotor = new WPI_TalonSRX(Constants.PICKUP_MOTOR_PORT);
    armMotor = new WPI_TalonSRX(Constants.ARM_MOTOR_PORT);
    armMotor.config_kP(0, Constants.ARM_KP);
    armMotor.configSelectedFeedbackCoefficient(360. / 800.);
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
   * @param angle 0 is the starting position, positive means more down.
   */
  public void setArm(int angle) {
    armMotor.set(ControlMode.Position, angle);
  }

  /**
   * 
   * @return A StartEndCommand that starts the pickup.
   */
  public StartEndCommand getPickupCommand() {
    return new StartEndCommand(this::startPickup, this::stopPickup, this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Arm Position", this::getArmPosition, null);
  }
}
