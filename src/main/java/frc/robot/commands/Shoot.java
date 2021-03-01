/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooting;

public class Shoot extends CommandBase {
  private Shooting shooting;
  private double vel, angle;
  private DoubleSupplier velGetter, angleGetter;

  public Shoot(Shooting shooting, double velocity, double angle) {
    addRequirements(shooting);
    this.shooting = shooting;
    this.vel = velocity;
    this.angle = angle;
  }

  public Shoot(Shooting shooting, DoubleSupplier velGetter, DoubleSupplier angleGetter) {
    this.velGetter = velGetter;
    this.angleGetter = angleGetter;
  }

  public static enum ShootType {
    onClick, onHold
  }

  @Override
  public void initialize() {
    shooting.setBonk(0);
    shooting.setVacuum(false);
  }

  @Override
  public void execute() {
    shooting.setHoodAngle(angleGetter.getAsDouble());
    shooting.setWheelVel(velGetter.getAsDouble());
    if (shooting.getMaxLim()) {
      shooting.setBonk(0);
    } else if (Math.abs(shooting.getWheelVel() - vel) <= 0.01
        && Math.abs(shooting.getHoodAngle() - angle) <= 0.1) {
          if (!shooting.getMaxLim()) shooting.setBonk(1);
          shooting.setVacuum(true);
        }
  }

  @Override
  public void end(boolean interrupted) {
    shooting.setHoodAngle(0);
    shooting.setWheelVel(0);
    shooting.setVacuum(false);
    Bonk bonk = new Bonk(shooting, false);
    bonk.schedule();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
