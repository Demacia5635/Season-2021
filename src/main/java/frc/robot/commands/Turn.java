/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * An example command that uses an example subsystem.
 */
public class Turn extends CommandBase {
    private final Chassis chassis;
    private double setPosLeft;
    private double setPosRight;
    private double angle;
    private boolean forward; 
    private XboxController controller;

    // according to the documentaion:
    // After gain/settings are determined,
    // the robot-application only needs to periodically set the target position.
    // so we should check if the distance should be the same o do we need to lower
    // it as we move
    // also: Three additional parameters need to be set in the Talon SRX:
    // Acceleration, Cruise Velocity, and Acceleration Smoothing.
    // example for the config: configMotionAcceleration (double
    // sensorUnitsPer100msPerSec)
    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public Turn(Chassis chassis, double angle, XboxController controller) {
        this.chassis = chassis;
        // Use addRequirements() here to declare subsystem dependencies.
        this.angle = angle;
        this.controller = controller;
        addRequirements(chassis);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.setPosLeft = chassis.getLeftPos();
        this.setPosRight = chassis.getRightPos();
        this.forward = controller.getY(Hand.kLeft) > 0.2; 
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double distance = (angle * 2 * Math.PI * Constants.ROBOT_TRACK_WIDTH / 360); 
        if (this.forward) {
            if (this.angle > 0) this.chassis.setPosLeft(this.setPosLeft + distance); 
            else this.chassis.setPosRight(this.setPosRight + distance); 
        }
        else {
            if (this.angle > 0) this.chassis.setPosRight(this.setPosRight - distance); 
            else this.chassis.setPosLeft(this.setPosLeft - distance); 
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        double distance = (angle * 2 * Math.PI * Constants.ROBOT_TRACK_WIDTH / 360); 
        if (this.forward) {
            if (this.angle > 0) return this.chassis.getLeftPos() > 0.95 * (this.setPosLeft + distance)
            && this.chassis.getLeftPos() < 1.05 * (this.setPosLeft + distance); 
            else return this.chassis.getRightPos() > 0.95 * (this.setPosRight + distance)
            && this.chassis.getRightPos() < 1.05 * (this.setPosRight + distance); 
        }
        else {
            if (this.angle > 0) return this.chassis.getRightPos() > 0.95 * (this.setPosRight - distance)
            && this.chassis.getRightPos() < 1.05 * (this.setPosRight - distance); 
            else return this.chassis.getLeftPos() > 0.95 * (this.setPosLeft - distance)
            && this.chassis.getLeftPos() < 1.05 * (this.setPosLeft - distance); 
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {

    }

}
