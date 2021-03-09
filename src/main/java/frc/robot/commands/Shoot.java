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
  private DoubleSupplier velGetter, angleGetter;

  public Shoot(Shooting shooting, DoubleSupplier velGetter, DoubleSupplier angleGetter) {
    this.velGetter = velGetter;
    this.angleGetter = angleGetter;
    this.shooting = shooting;
  }

  public static enum ShootType {
    onClick, onHold
  }

  @Override
  public void initialize() {
    shooting.stopBonk();
    shooting.setVacuum(false);
  }

  @Override
  public void execute() {
    //shooting.setHoodAngle(angleGetter.getAsDouble());
    shooting.setWheelVel(velGetter.getAsDouble());
    if (Math.abs(shooting.getWheelVel() - velGetter.getAsDouble()) <= 150
        /*&& true/*Math.abs(shooting.getHoodAngle() - angleGetter.getAsDouble()) <= 0.1*/) {
          if (shooting.getForwardSwitch() == 0){
            shooting.bonkUp();
          } else {
            shooting.stopBonk();
          }
          
          
        }
        if (Math.abs(shooting.getWheelVel() - velGetter.getAsDouble()) <= 400){
          shooting.setVacuum(true);
        }
  }

  @Override
  public void end(boolean interrupted) {
    //shooting.setHoodAngle(0);
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
