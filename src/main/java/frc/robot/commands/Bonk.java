// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooting;

public class Bonk extends CommandBase {
  private Shooting shooting;
  private boolean up;
  private int count;

  public Bonk(Shooting shooting, boolean up) {
    addRequirements(shooting);
    this.shooting = shooting;
    this.up = up;
  }

  @Override
  public void initialize() {
    if (up){
      shooting.bonkUp();
    } else shooting.bonkDown();
    count = 0;
  }

  @Override
  public void execute() {
    count++;
  }

  @Override
  public void end(boolean interrupted) {
    shooting.stopBonk();
  }

  @Override
  public boolean isFinished() {
    if (up){
      return shooting.getForwardSwitch() == 1;
    } else return shooting.getReverseSwitch() == 1;
  }
}
