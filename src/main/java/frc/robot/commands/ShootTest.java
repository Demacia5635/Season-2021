// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooting;
import frc.robot.subsystems.Chassis;

public class ShootTest extends CommandBase {
  private Shooting shooting;
  private Chassis chassis;

  public ShootTest(Shooting shooting, Chassis chassis) {
    this.shooting = shooting;
    this.chassis = chassis;
  }

  @Override
  public void initialize() {
    SmartDashboard.setDefaultNumber("SVelocity", 4500);
    SmartDashboard.setDefaultNumber("SAngle", 5);
    SmartDashboard.setDefaultBoolean("Shoot", false);
    SmartDashboard.setDefaultBoolean("Save", false);
  }

  @Override
  public void execute() {
    if (SmartDashboard.getBoolean("Shoot", false)) {
      shoot();
      SmartDashboard.putBoolean("Shoot", false);
    } else if (SmartDashboard.getBoolean("NextPos", false)) {
      nextPos();
      SmartDashboard.putBoolean("NextPos", false);
    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private double getVelocity(){
    return SmartDashboard.getNumber("SVelocity", 4500);
  }

  private double getAngle(){
    return SmartDashboard.getNumber("SAngle", 5);
  }

  private void shoot() {
    new Shoot(shooting, this::getVelocity, this::getAngle).withTimeout(15).schedule();
  }

  private void nextPos() {
    new GoTo(chassis, -0.5);
  }
}
