/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooting extends SubsystemBase {
  private TalonSRX bigWheel;
  private TalonSRX bonker;
  private TalonSRX hoodMotor;
  public Shooting() {
    bigWheel = new TalonSRX(Constants.SHOOTER_WHEEL_PORT);
    bonker = new TalonSRX(Constants.BONKER_PORT);
    hoodMotor = new TalonSRX(Constants.HOOD_MOTOR_PORT);
    hoodMotor.configSelectedFeedbackCoefficient(360./Constants.PULSE_PER_ROTATION);
    bigWheel.configSelectedFeedbackCoefficient(Constants.PULSE_PER_ROTATION/(Constants.SHOOTER_DIAMETER*Math.PI));
  }

  public void setHoodAngle(double angle){// in degrees
    hoodMotor.set(ControlMode.Position, angle);
  }

  public void setWheelVel(double v){// in m/s
    bigWheel.set(ControlMode.Velocity, v/10., DemandType.ArbitraryFeedForward, Constants.SHOOTER_KS*v);
  }

  public double getWheelVel(){
    return bigWheel.getSelectedSensorVelocity()*10.;
  }

  public double getHoodAngle(){
    return hoodMotor.getSelectedSensorVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
