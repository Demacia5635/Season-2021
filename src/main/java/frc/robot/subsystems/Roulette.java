/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Roulette extends SubsystemBase {

  private WPI_TalonSRX roulette;
  private double rouletteVel;

  public Roulette() {
    setRouletteVel(-20);
    roulette = new WPI_TalonSRX(Constants.ROULETTE_MOTOR_PORT);
    roulette.setSelectedSensorPosition(0);
    roulette.config_kP(0, 5);
    roulette.config_kI(0, 0.001);
    roulette.config_kD(0, 0.0001);
    roulette.setSensorPhase(true);
    setDefaultCommand(getSpinCommand());
  }

  public void startSpin() {
    roulette.set(ControlMode.Velocity, rouletteVel, DemandType.ArbitraryFeedForward, Math.signum(rouletteVel) * 0.1);
  }

  public void stopSpin() {
    roulette.set(ControlMode.PercentOutput, 0);
  }

  /**
   * @return A StartEndCommand that starts the roulette's spin
   */
  public Command getSpinCommand() {
    return new RunCommand(this::startSpin,this).andThen(new InstantCommand(this::stopSpin));
  }

  public double getEncoder(){
    return roulette.getSelectedSensorPosition();
  }

  public void setRouletteVel(double vel){
    rouletteVel = vel;
  }

  @Override
  public void periodic() {
  }
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Encoder", this::getEncoder, null);
   //roulette.initSendable(builder);
   builder.addDoubleProperty("Roulette Error", roulette::getClosedLoopError, null);
   builder.addDoubleProperty("Roulette Power", roulette::get, null);
    
    //builder.addDoubleProperty("Velocity", this::, setter);
  }
}
