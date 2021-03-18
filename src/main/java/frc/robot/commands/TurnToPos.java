// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

public class TurnToPos extends CommandBase {

  private Chassis chassis;
  private TurnType type;
  private DoubleSupplier angleGetter;
  private Turn turnCommand;

  public TurnToPos(Chassis chassis, TurnType type, DoubleSupplier angleGetter) {
    this.chassis = chassis;
    this.type = type;
    this.angleGetter = angleGetter;
    switch (type){
      case Passive:
        addRequirements(chassis);
        break;
      default:
        break;
    }
  }

  public static enum TurnType {
    Active, Passive
  }

  @Override
  public void initialize() {
    switch (type) {
      case Passive:
        chassis.setVelocity(1, -1);
        break;
      case Active:
        turnCommand = new Turn(chassis, angleGetter);
        turnCommand.schedule();
      default:
        break;
    }
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    switch (type){
      case Passive:
        chassis.setVelocity(0, 0);
        break;
      case Active:
        turnCommand.cancel();
        break;
    }
    
  }

  @Override
  public boolean isFinished() {
    switch (type) {
      case Passive:
        return angleGetter.getAsDouble() != 0;
      default:
        return false;
    }
  }

}
