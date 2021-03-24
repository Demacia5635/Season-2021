// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.utils.FeedForward;
import frc.robot.subsystems.Chassis;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/** An example command that uses an example subsystem. */
public class Calibrate extends CommandBase {


  public static final double Power1 = 0.4;
  public static final double Power2 = 0.3;
  public static final double MinPower = 0.1;

  Chassis chassis;
  CalibrateTurn[] cmds;
  Command cmd;
  

  public Calibrate(Chassis chassis) {
    super();
    this.chassis = chassis;
    // create the list of Calibrate Turn Commands
    cmds = new CalibrateTurn[] {
      new CalibrateTurn(chassis, chassis::setPower, chassis::getLeftVelocity, chassis::getRightVelocity, Power1, Power1),
      new CalibrateTurn(chassis, chassis::setPower, chassis::getLeftVelocity, chassis::getRightVelocity, -Power1, -Power1),
      new CalibrateTurn(chassis, chassis::setPower, chassis::getLeftVelocity, chassis::getRightVelocity, Power1, MinPower),
      new CalibrateTurn(chassis, chassis::setPower, chassis::getLeftVelocity, chassis::getRightVelocity, -Power1, -MinPower),
      new CalibrateTurn(chassis, chassis::setPower, chassis::getLeftVelocity, chassis::getRightVelocity, Power2, Power2),
      new CalibrateTurn(chassis, chassis::setPower, chassis::getLeftVelocity, chassis::getRightVelocity, -Power2, -Power2),
      new CalibrateTurn(chassis, chassis::setPower, chassis::getLeftVelocity, chassis::getRightVelocity, Power2, MinPower),
      new CalibrateTurn(chassis, chassis::setPower, chassis::getLeftVelocity, chassis::getRightVelocity, -Power2, -MinPower)
    };
    // build the cmd to run all tests - with a 1 second wait in between
    cmd = null;
    for(Command c : cmds) {
      if(cmd == null) {
        cmd  = c;
      } else {
        // cmd = cmd.andThen().andThen(c);
        cmd = cmd.andThen(new WaitCommand(1)).andThen(c);
        System.out.println("RUNNING OVER THE COMMANDS WHICH ARENT");
      }
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // run the commnand
    cmd.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // nothing - waiting for the cmd o finish
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      // analyze the data
      // get the velocities - v1 = Power1/Power1 v2 = Power2/Power2, v1h = high of Power1/MinPower1 etc.
    double v1 = (cmds[0].lVelocity - cmds[1].lVelocity + cmds[0].rVelocity - cmds[1].rVelocity)/4;
    double v2 = (cmds[4].lVelocity - cmds[5].lVelocity + cmds[4].rVelocity - cmds[5].rVelocity)/4;
    double v1h = (cmds[2].lVelocity - cmds[3].lVelocity)/2;
    double v1l = (cmds[2].rVelocity - cmds[3].rVelocity)/2;
    double v2h = (cmds[6].lVelocity - cmds[7].lVelocity)/2;
    double v2l = (cmds[6].rVelocity - cmds[7].rVelocity)/2;

    // calculate the constants
    double ks = (Power1 - Power2) / (v1 - v2);
    double kv = ((Power1 - v1*ks) + (Power2 - v2*ks))/2;
    double hr = ((Power1 - MinPower)/(v1-v1h) + (Power2 - MinPower)/v2-v2h)/2;
    double lr = ((Power1 - MinPower)/(v1-v1l) + (Power2 - MinPower)/v2-v2l)/2;
    // set up the global parameters - for the test
    FeedForward.K_S = ks;
    FeedForward.K_V = kv;
    FeedForward.K_HA = hr;
    FeedForward.K_LA = lr;
    // check all values
    double error = 0;
    for(CalibrateTurn c : cmds) {
      double pl = FeedForward.feedForwardLeftPower(c.lVelocity, c.rVelocity);
      double pr = FeedForward.feedForwardRightPower(c.lVelocity, c.rVelocity);
      error = Math.abs(pl - c.lPower) + Math.abs(pr - c.rPower);
    }
    // put all data into the network table
    SmartDashboard.putNumber("Calibrate KS", ks);
    SmartDashboard.putNumber("Calibrate KV", kv);
    SmartDashboard.putNumber("Calibrate HR", hr);
    SmartDashboard.putNumber("Calibrate LR", lr);
    SmartDashboard.putNumber("Calibrate Error", error/16); // average of the 16 tests
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return cmd.isFinished();
  }
}