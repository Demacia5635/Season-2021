/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooting extends SubsystemBase {

  private WPI_TalonSRX bigWheel;
  private WPI_TalonSRX bonker;
  private WPI_TalonSRX hoodMotor;
  private WPI_TalonSRX vacuumMotor;
  private int hoodSwitchLastCycle;
  private boolean vacuumState;// true if on, false if off

  public Shooting() {
    bigWheel = new WPI_TalonSRX(Constants.SHOOTER_WHEEL_PORT);
    bonker = new WPI_TalonSRX(Constants.BONKER_PORT);
    hoodMotor = new WPI_TalonSRX(Constants.HOOD_MOTOR_PORT);
    vacuumMotor = new WPI_TalonSRX(Constants.VACUUM_MOTOR_PORT);
    bigWheel.setInverted(InvertType.InvertMotorOutput);
    hoodMotor.setSensorPhase(true);
    hoodMotor.setInverted(true);
    bigWheel.setSensorPhase(true);
    hoodMotor.configForwardSoftLimitThreshold(50 * 800 / 360);
    hoodMotor.configForwardSoftLimitEnable(true);
    bigWheel.config_kP(0, Constants.SHOOTER_KP);
    hoodMotor.config_kP(0, Constants.HOOD_KP);
    hoodMotor.config_kI(0, Constants.HOOD_KI);
    hoodSwitchLastCycle = (int) getHoodLimit();
    bigWheel.configContinuousCurrentLimit(20);
    bigWheel.enableCurrentLimit(true);
    bonker.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
        LimitSwitchNormal.NormallyOpen);
    bonker.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
        LimitSwitchNormal.NormallyOpen);
    hoodMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
        LimitSwitchNormal.NormallyOpen);
    vacuumState = false;
    
    setDefaultCommand(new RunCommand(this::getHoodBack, this));
  }

  /**
   * Sets the angle of the hood motor.
   * 
   * @param angle The desired angle of the hood 0 means it's original position.
   */
  public void setHoodAngle(double angle) {
    if (getHoodLimit() == 0 || angle > 0) {
      hoodMotor.set(ControlMode.Position, angle * 800. / 360.);
    }

  }

  /**
   * Sets the shooting wheel's velocity.
   * 
   * @param v In degrees/sec.
   */
  public void setWheelVel(double v) {
    double feedforward = Constants.SHOOTER_KV + Constants.SHOOTER_KS * v;
    if (v == 0){
      feedforward = 0;
    }
    bigWheel.set(ControlMode.Velocity, v / 10. * 800. / 360., DemandType.ArbitraryFeedForward,
        feedforward);
  }
  
  public void bonkUp(){
    bonker.set(ControlMode.PercentOutput, 0.15);
  }

  public void bonkDown(){
    bonker.set(ControlMode.PercentOutput, -0.15);
  }

  public void stopBonk(){
    bonker.set(ControlMode.PercentOutput, 0.);
  }

   public void setHood(double percent){
    hoodMotor.set(ControlMode.PercentOutput, percent);
  }

  /**
   * Sets the vacuum's state.
   * 
   * @param on true to make balls go in, false to keep them in the roulette.
   */
  public void setVacuum(boolean on) {
    if (vacuumState == on) return;
    vacuumMotor.set(ControlMode.PercentOutput, on ? 1 : 0);
    vacuumState = on;
  }

  /**
   * 
   * @return The velocity of the shooter wheel in degrees/sec.
   */
  public double getWheelVel() {
    return bigWheel.getSelectedSensorVelocity() * 10. * 360. / 800.;
  }

  public RunCommand getHoodCommand() {
    return new RunCommand(this::setHoodOn, this);
  }

  public void setHoodOn() {
    // setHoodAngle(10);
    if (/*getHoodLimit() == 0*/ getHoodAngle() < 20 && getHoodAngle() < 50) {
      hoodMotor.set(ControlMode.PercentOutput, 0.4);
    } else {
      setHoodOff();
      System.out.println("stopped");
    } 

  }

  public void setWheel(double percent){
    bigWheel.set(ControlMode.PercentOutput, percent);
  }

  public void getHoodBack() {
    if (getHoodLimit() == 0) {
      hoodMotor.set(ControlMode.PercentOutput, -0.1);
    } else {
      System.out.println("stopped");
      setHoodOff();
    } 
  }

  public void setHoodOff() {
    hoodMotor.set(ControlMode.PercentOutput, 0);
  }

  /**
   * 
   * @return The angle of the hood in degrees.
   */
  public double getHoodAngle() {
    return hoodMotor.getSelectedSensorPosition() * 360. / 800.;
  }

  /**
   * 
   * @return The state of the vacuum true if on, false if off.
   */
  public boolean getVacuumState() {
    return vacuumState;
  }

  @Override
  public void periodic() {
    if (hoodSwitchLastCycle == 0 && getHoodLimit() == 1) {
      resetHoodEncoder();
    }
    hoodSwitchLastCycle = (int) getHoodLimit();
  }

  public double getForwardSwitch() {
    return bonker.isFwdLimitSwitchClosed();
  }

  public double getReverseSwitch() {
    return bonker.isRevLimitSwitchClosed();
  }

  public double getHoodLimit() {
    return hoodMotor.isRevLimitSwitchClosed();
  }

  public void setWheelpower() {
    setWheelVel(6000);
    setVacuum(true);
  }

  public void resetHoodEncoder() {
    hoodMotor.setSelectedSensorPosition(0);
  }

  public void stopWheel() {
    bigWheel.set(ControlMode.PercentOutput, 0);
    setVacuum(false);
  }

  public StartEndCommand getshootercmd() {
    return new StartEndCommand(this::setWheelpower, this::stopWheel, this);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Wheel Velocity", this::getWheelVel, null);
    builder.addDoubleProperty("Hood Angle", this::getHoodAngle, null);
    builder.addDoubleProperty("Bonk forward swtich", this::getForwardSwitch, null);
    builder.addDoubleProperty("Bonk reverse swtich", this::getReverseSwitch, null);
    builder.addDoubleProperty("Hood limit", this::getHoodLimit, null);
    hoodMotor.initSendable(builder);
  }

}
