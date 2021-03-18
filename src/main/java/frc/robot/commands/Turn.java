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

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

/**
 * An example command that uses an example subsystem.
 */
public class Turn extends CommandBase {
    private final double errorRange = 0.003; // In meters
    private double destination; // In Pulses
    private double distance;
    private final Chassis chassis;
    private double startPosLeft;
    private double startPosRight;
    private double angle;
    private DoubleSupplier angleGetter;

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
        this.distance = Math.abs(angle) * (2 * Math.PI * Constants.ROBOT_TRACK_WIDTH) / 360.0;
        addRequirements(chassis);
    }

    public Turn(Chassis chassis, DoubleSupplier angleGetter){
        this.chassis = chassis;
        this.angleGetter = angleGetter;
        addRequirements(chassis);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (angleGetter != null){
            angle = angleGetter.getAsDouble();
            distance = Math.abs(angle) * (2 * Math.PI * Constants.ROBOT_TRACK_WIDTH) / 360.0;
        }
        startPosLeft = chassis.getLeftPos();
        startPosRight = chassis.getRightPos();
        destination = (angle > 0 ? startPosRight : startPosLeft) - distance;
        chassis.Set_K_I(0);
        chassis.Set_K_D(0);
        chassis.Set_K_P((Constants.CHASSIS_KV / 12) / 480 / 20);
        chassis.setAllowedError(150);
        if (angle > 0) {
            chassis.setRightPos(destination, -Constants.CHASSIS_KV / 12);
            chassis.setLeftPos(startPosLeft);
        } 
        else {
            chassis.setLeftPos(destination, -Constants.CHASSIS_KV / 12);
            chassis.setRightPos(startPosRight);
        }
        // 0.0145
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    public double getDistance() {
        return distance;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        chassis.setPower(0, 0);
        chassis.Set_K_I(Constants.CHASSIS_KI);
        chassis.Set_K_D(Constants.CHASSIS_KD);
        chassis.Set_K_P(Constants.CHASSIS_KP);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        System.out.println("right pos " + chassis.getRightPos());
        System.out.println("left pos " + chassis.getLeftPos());
        System.out.println("diff " + Math.abs(chassis.getRightPos() - destination));
        if (angleGetter == null){
            return angle > 0 ? chassis.getRightPos() - destination < errorRange
                : chassis.getLeftPos() - destination < errorRange;
        } else {
            return false;
        }
        
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("angle distance", this::getDistance, null);
    }

}
