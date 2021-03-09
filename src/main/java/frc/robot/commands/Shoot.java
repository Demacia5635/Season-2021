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
  private double angle;
  private boolean up;

  public Shoot(Shooting shooting, DoubleSupplier velGetter, DoubleSupplier angleGetter) {
    this.velGetter = velGetter;
    this.angleGetter = angleGetter;
    this.shooting = shooting;
  }

  @Override
  public void initialize() {
    shooting.stopBonk();
    shooting.setVacuum(false);
    angle = angleGetter.getAsDouble();
    up = angle > shooting.getHoodAngle();
  }

  @Override
  public void execute() {
    if (up && shooting.getHoodAngle() < angle){
      shooting.setHood(0.4 * Math.cos(Math.toRadians(shooting.getHoodAngle())));
    }
    else if (!up && shooting.getHoodAngle() > angle) shooting.setHood(0.4 * Math.cos(Math.toRadians(shooting.getHoodAngle())));
    else shooting.setHood(0);

    
    if (/*Math.abs(*/velGetter.getAsDouble() - shooting.getWheelVel() /*)*/ <= 150
        /*&& Math.abs(shooting.getHoodAngle() - angleGetter.getAsDouble()) <= 3*/) {
          if (shooting.getForwardSwitch() == 0){
            shooting.bonkUp();
          } else {
            shooting.stopBonk();
          }
          
          
        }
        if (/*Math.abs(*/ velGetter.getAsDouble() - shooting.getWheelVel() /*)*/ <= 400){
          shooting.setVacuum(true);
          shooting.setWheelVel(velGetter.getAsDouble());
        } else {
          shooting.setWheel(1);
        }
  }

  @Override
  public void end(boolean interrupted) {
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
