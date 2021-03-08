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

public class Roulette extends SubsystemBase {

  private WPI_TalonSRX roulette;

  public Roulette() {
    roulette = new WPI_TalonSRX(Constants.ROULETTE_MOTOR_PORT);
    roulette.setSelectedSensorPosition(0);
    roulette.config_kP(0, 5);
    roulette.config_kI(0, 0.001);
    roulette.config_kD(0, 0.0001);
    roulette.setSensorPhase(true);
    setDefaultCommand(getSpinCommand());
  }

  public void startSpin() {
    roulette.set(ControlMode.Velocity, -40);
  }

  public void stopSpin() {
    roulette.set(ControlMode.PercentOutput, 0);
  }

  /**
   * @return A StartEndCommand that starts the roulette's spin
   */
  public StartEndCommand getSpinCommand() {
    return new StartEndCommand(this::startSpin, this::stopSpin, this);
  }

  public double getEncoder(){
    return roulette.getSelectedSensorPosition();
  }

  @Override
  public void periodic() {
  }
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Encoder", this::getEncoder, null);
   //roulette.initSendable(builder);
    
    //builder.addDoubleProperty("Velocity", this::, setter);
  }
}
