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
  private double startAngle;

  public TurnToPos(Chassis chassis, TurnType type, DoubleSupplier angleGetter) {
    this.chassis = chassis;
    this.type = type;
    this.angleGetter = angleGetter;
    addRequirements(chassis);
  }

  public static enum TurnType {
    Active, Passive
  }

  @Override
  public void initialize() {
    startAngle = chassis.getFusedHeading();
    switch (type) {
      case Passive:
        // Turn by angle
        break;
      default:
        break;
    }
  }

  @Override
  public void execute() {
    switch (type) {
      case Active:
        // Turn by angle
        break;
      default:
        break;
    }
  }

  @Override
  public void end(boolean interrupted) {
    chassis.setVelocity(0, 0);
  }

  @Override
  public boolean isFinished() {
    switch (type) {
      case Passive:
        return Math.abs(angleGetter.getAsDouble()) <= 1;
      default:
        return false;
    }
  }

}
