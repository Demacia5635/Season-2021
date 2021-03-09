// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooting;

public class HoodMove extends CommandBase {
  private Shooting shooting;
  private boolean up;
  private double angle;

  public HoodMove(Shooting shooting, boolean up) {
    addRequirements(shooting);
    this.shooting = shooting;
    this.up = up;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angle = shooting.getHoodAngle() + (up ? 2 : -2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooting.setHood(0.4 * Math.cos(Math.toRadians(shooting.getHoodAngle())));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooting.setHood(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (up){
      return shooting.getHoodAngle() >= angle;
    }
    else return shooting.getHoodAngle() <= angle;
  }
}
