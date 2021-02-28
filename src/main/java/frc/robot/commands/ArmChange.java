// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pickup;

public class ArmChange extends CommandBase {

  private Pickup pickup;
  private Position pos;

  public enum Position {
    Top, Bottom, Middle
  }

  public ArmChange(Position pos, Pickup pickup) {
    this.pickup = pickup;
    this.pos = pos;
    addRequirements(pickup);
  }

  @Override
  public void initialize() {
    switch (pos) {
      case Top:
        pickup.setArm(0);
        break;
      case Bottom:
        pickup.setArm(90);
        break;
      case Middle:
        pickup.setArm(45);
        break;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    switch (pos) {
      case Top:
        return Math.abs(pickup.getArmPosition()) <= 2;
      case Bottom:
        return Math.abs(pickup.getArmPosition() - 90) <= 2;
      case Middle:
        return Math.abs(pickup.getArmPosition() - 45) <= 2;
      default:
        return false;
    }
  }
}
