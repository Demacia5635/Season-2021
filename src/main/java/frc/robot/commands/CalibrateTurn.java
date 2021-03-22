// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Chassis;
import java.util.function.BiConsumer;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class CalibrateTurn extends CommandBase {

  public static double RunTime = 2.0;

  public double lPower;
  public double rPower;
  public double lVelocity;
  public double rVelocity;

  private Chassis chassis;
  private BiConsumer<Double, Double> setPower;
  private Supplier<Double> getLeftVelocity;
  private Supplier<Double> getRightVelocity;
  private Timer timer;

  public CalibrateTurn(Chassis chassis,
                BiConsumer<Double, Double> setPower,
                Supplier<Double> getLeftVelocity,
                Supplier<Double> getRightVelocity,
                double lPower,
                double rPower) {
    super();
    this.chassis = chassis;
    this.addRequirements(this.chassis);
    this.setPower = setPower;
    this.getLeftVelocity = getLeftVelocity;
    this.getRightVelocity = getRightVelocity;
    timer = new Timer();
    this.lPower = lPower;
    this.rPower = rPower;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    setPower.accept(lPower, rPower);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    lVelocity = getLeftVelocity.get();
    rVelocity = getRightVelocity.get();
    setPower.accept(0.,0.);
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(RunTime);
  }
}