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
    bigWheel = new TalonSRX(Constants.shooterWheelPort);
    bonker = new TalonSRX(Constants.bonkerPort);
    hoodMotor = new TalonSRX(Constants.hoodMotorPort);
    vacuumMotor = new TalonSRX(Constants.vacuumMotor);
    max = new DigitalInput(Constants.maxLim);
    min = new DigitalInput(Constants.minLim);
    hoodMotor.configSelectedFeedbackCoefficient(360./Constants.pulsePerRotation);
    bigWheel.configSelectedFeedbackCoefficient(360./Constants.pulsePerRotation);
    vacuumState = false;
  }

  public void setHoodAngle(double angle){// in degrees
    hoodMotor.set(ControlMode.Position, angle);
  }

  public void setWheelVel(double v){// in angle/sec
    bigWheel.set(ControlMode.Velocity, v/10., DemandType.ArbitraryFeedForward, Constants.shooterKS*v);
  }

  public void setBonk(double percent){//percent of the power
    bonker.set(ControlMode.PercentOutput, percent);
  }

  public void setVacuum(boolean on){// true to make balls go in, false to keep them going in the roulette
    if (vacuumState == on) return;
    vacuumMotor.set(ControlMode.PercentOutput, on ? 1 : 0);
    vacuumState = on;
  }

  public double getWheelVel(){
    return bigWheel.getSelectedSensorVelocity()*10.;
  }

  public double getHoodAngle(){
    return hoodMotor.getSelectedSensorVelocity();
  }

  public boolean getMaxLim(){
    return max.get();
  }

  public boolean getMinLim(){
    return min.get();
  }

  public boolean getVacuumState(){
    return vacuumState;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
