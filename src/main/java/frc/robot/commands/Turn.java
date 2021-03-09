/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Chassis;
import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

/**
 * An example command that uses an example subsystem.
 */
public class Turn extends CommandBase {
    private final double errorRange = 0.05; // In meters
    private final double destination; // In Pulses
    private final double distance;
    private final Chassis chassis;
    private double startPosLeft;
    private double startPosRight;
    private double angle;

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
    public Turn(Chassis chassis, double angle) {
        this.chassis = chassis;
        // Use addRequirements() here to declare subsystem dependencies.
        this.angle = angle;
        distance = angle * (2 * Math.PI * Constants.ROBOT_TRACK_WIDTH) / 360;
        destination = (angle > 0 ? startPosRight : startPosLeft) - distance;
        addRequirements(chassis);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        startPosLeft = chassis.getLeftPos();
        startPosRight = chassis.getRightPos();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (angle > 0) chassis.setRightPos(destination);
        else chassis.setLeftPos(destination);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return angle > 0 ? Math.abs(chassis.getRightPos() - destination) < errorRange
                : Math.abs(chassis.getLeftPos() - destination) < errorRange;
    }

    @Override
    public void initSendable(SendableBuilder builder) {

    }

}
