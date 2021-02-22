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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooting extends SubsystemBase {

  private TalonSRX bigWheel;
  private TalonSRX bonker;
  private TalonSRX hoodMotor;
  private TalonSRX vacuumMotor;
  private DigitalInput max;
  private DigitalInput min;
  private boolean vacuumState;//true if on, false if off

  public Shooting() {
    bigWheel = new TalonSRX(Constants.SHOOTER_WHEEL_PORT);
    bonker = new TalonSRX(Constants.BONKER_PORT);
    hoodMotor = new TalonSRX(Constants.HOOD_MOTOR_PORT);
    vacuumMotor = new TalonSRX(Constants.VACUUM_MOTOR_PORT);
    max = new DigitalInput(Constants.MAX_LIM_PORT);
    min = new DigitalInput(Constants.MIN_LIM_PORT);
    hoodMotor.configSelectedFeedbackCoefficient(360./800.);
    bigWheel.configSelectedFeedbackCoefficient(360./800.);
    hoodMotor.config_kP(0, Constants.HOOD_KP);
    vacuumState = false;
  }

  /**
   * Sets the angle of the hood motor.
   * @param angle The desired angle of the hood 0 means it's original position.
   */
  public void setHoodAngle(double angle){
    hoodMotor.set(ControlMode.Position, angle);
  }

  /**
   * Sets the shooting wheel's velocity.
   * @param v In degrees/sec.
   */
  public void setWheelVel(double v){
    bigWheel.set(ControlMode.Velocity, v/10., DemandType.ArbitraryFeedForward, Constants.SHOOTER_KS*v);
  }

  /**
   * Sets the bonk's power.
   * @param percent Between 1 to -1.
   */
  public void setBonk(double percent){
    bonker.set(ControlMode.PercentOutput, percent);
  }

  /**
   * Sets the vacuum's state.
   * @param on true to make balls go in, false to keep them in the roulette.
   */
  public void setVacuum(boolean on){// true to make balls go in, false to keep them going in the roulette.
    if (vacuumState == on) return;
    vacuumMotor.set(ControlMode.PercentOutput, on ? 1 : 0);
    vacuumState = on;
  }

  /**
   * 
   * @return The velocity of the shooter wheel in degrees/sec.
   */
  public double getWheelVel(){
    return bigWheel.getSelectedSensorVelocity()*10.;
  }

  /**
   * 
   * @return The angle of the hood in degrees.
   */
  public double getHoodAngle(){
    return hoodMotor.getSelectedSensorPosition();
  }

  /**
   * Corresponeds to the upper limit of the bonker.
   * @return If the limit sensor is on on or off.
   */
  public boolean getMaxLim(){
    return max.get();
  }

  /**
   * Corresponeds to the lower limit of the bonker.
   * @return If the limit sensor is on on or off.
   */
  public boolean getMinLim(){
    return min.get();
  }

  /**
   * 
   * @return The state of the vacuum true if on, false if off. 
   */
  public boolean getVacuumState(){
    return vacuumState;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
