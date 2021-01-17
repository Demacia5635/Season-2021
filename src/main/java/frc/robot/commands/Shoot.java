/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooting;

public class Shoot extends CommandBase {
  private Shooting shooting;
  private double vel;
  private double angle;
  public Shoot(Shooting shooting, double velocity, double angle) {
    this.shooting = shooting;
    this.vel = velocity;
    this.angle = angle;
  }

  @Override
  public void initialize() {
    shooting.setHoodAngle(angle);
    shooting.setWheelVel(vel);
  }

  @Override
  public void execute() {
    if (Math.abs(shooting.getWheelVel() - vel) <= 0.01 && Math.abs(shooting.getHoodAngle() - angle) <= 0.1){
      
    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
