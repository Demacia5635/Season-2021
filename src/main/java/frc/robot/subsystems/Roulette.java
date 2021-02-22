/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Roulette extends SubsystemBase {

  private TalonSRX roulette;

  public Roulette() {
    roulette = new TalonSRX(Constants.ROULETTE_MOTOR_PORT);
  }

  public void startSpin() {
    roulette.set(ControlMode.PercentOutput, 1);
  }

  public void stopSpin() {
    roulette.set(ControlMode.PercentOutput, 0);
  }

  /**
   * @return A StartEndCommand that starts the roulette's spin
   */
  public StartEndCommand getSpinCommand(){
    return new StartEndCommand(this::startSpin, this::stopSpin, this);
  }

  @Override
  public void periodic() {
    
  }
}
