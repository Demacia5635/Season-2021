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
    SmartDashboard.setDefaultNumber("Velocity for shooting", 1);
    SmartDashboard.setDefaultNumber("Angle for shooting", 1);
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
    return SmartDashboard.getNumber("Velocity for shooting", 1);
  }

  private double getAngle(){
    return SmartDashboard.getNumber("Angle for shooting", 1);
  }

  private void shoot() {
    new Shoot(shooting, this::getVelocity, this::getAngle).withTimeout(10).schedule();
  }

  private void nextPos() {
    new GoTo(chassis, -0.5);
  }
}
