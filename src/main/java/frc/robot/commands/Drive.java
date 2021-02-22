/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Chassis;
// some debugging power
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drive extends CommandBase implements Sendable{

  public enum InputHandler {
    tank, singer, triggerTurns, YandX
  }

  public enum DriveStates {
    curvatureDrive, arcadeDrive, angularVelocity, radialAccelaration
  }

  private final Chassis chassis;
  private XboxController controller;
  private InputHandler inputHandler;
  private DriveStates driveState;
  private double velocity;
  private double turns;

  /**
   * Creates a new Drive.
   */
  public Drive(Chassis chassis, InputHandler inputHandler, DriveStates driveState) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.chassis = chassis;
    controller = new XboxController(Constants.xboxPort);
    this.inputHandler = inputHandler;
    this.driveState = driveState;
    this.velocity = 0;
    this.turns = 0;
    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.velocity = 0;
    this.turns = 0;
    switch (inputHandler) {
      case YandX:
        velocity = controller.getY(Hand.kLeft);
        turns = controller.getX(Hand.kRight);
        break;
      case tank:
        chassis.setVelocity(controller.getY(Hand.kRight) * Constants.maxVelocity,
            controller.getY(Hand.kLeft) * Constants.maxVelocity);
        break;
      case singer:
        velocity = controller.getTriggerAxis(Hand.kRight) - controller.getTriggerAxis(Hand.kLeft);
        turns = controller.getX(Hand.kLeft);
        break;
      case triggerTurns:
        velocity = controller.getY(Hand.kLeft);
        turns = controller.getTriggerAxis(Hand.kRight) - controller.getTriggerAxis(Hand.kLeft);
        break;
    }
    if (inputHandler != InputHandler.tank) {
      switch (driveState) {
        case arcadeDrive:
          chassis.arcadeDrive(velocity, turns, true);
          break;
        case curvatureDrive:
          boolean isQuickTurn = controller.getBumper(Hand.kRight);
          chassis.curvatureDrive(velocity, turns, isQuickTurn);
          break;
        case radialAccelaration:
          if (Math.abs(velocity) < 0.02) {
            velocity = 0;
          }
          if (Math.abs(turns) < 0.02) {
            turns = 0;
          }
          chassis.radialAccelaration(velocity, turns);
          break;
        case angularVelocity:
          if (Math.abs(velocity) < 0.02) {
            velocity = 0;
          }
          if (Math.abs(turns) < 0.02) {
            turns = 0;
          }
          chassis.angularVelocity(velocity, turns);
          break;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    // builder.addDoubleProperty(key, getter, setter);
  }
}
