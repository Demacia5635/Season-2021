// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

public class TurnByVel extends CommandBase {
  /** Creates a new TurnByVel. */
  private final double angle; 
  private Chassis chassis; 
  private double startAngle; 


  public TurnByVel(Chassis chassis, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.chassis = chassis; 
    this.angle = angle; 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startAngle = chassis.getAngle();
    if (angle < 0) {
      chassis.setVelocityOurFF(0, -0.1);
    }
    else {
      chassis.setVelocityOurFF(-0.1, 0);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.setPower(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.printf(" angle=%f start=%f getAngle=%f\n",angle, startAngle, chassis.getAngle());
    if (angle > 0) {
      return chassis.getAngle() - startAngle > angle - 0.4;
    }
    else {
      return chassis.getAngle() - startAngle < angle + 0.4;
    }
  }
}
