/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import frc.robot.Constants;

public class GroupOfMotors {
    private TalonSRX lead;
    private TalonSRX[] followers;

    public GroupOfMotors(int... talons) {
        lead = new WPI_TalonSRX(talons[0]);
        lead.config_kP(0, Constants.kp);
        lead.config_kD(0, Constants.kd);
        lead.setNeutralMode(NeutralMode.Brake);
        lead.enableCurrentLimit(true);
        followers = new TalonSRX[talons.length - 1];
        for (int i = 0; i < followers.length; i++) {
            followers[i] = new WPI_TalonSRX(talons[i + 1]);
            followers[i].setNeutralMode(NeutralMode.Brake);
            followers[i].follow(lead);
        }
    }

    public GroupOfMotors(WPI_TalonSRX... talons) {
        lead = talons[0];
        lead.config_kP(0, Constants.kp);
        lead.config_kD(0, Constants.kd);
        lead.setNeutralMode(NeutralMode.Brake);
        lead.enableCurrentLimit(true);
        followers = new TalonSRX[talons.length - 1];
        for (int i = 0; i < followers.length; i++) {
            followers[i] = talons[i + 1];
            followers[i].setNeutralMode(NeutralMode.Brake);
            followers[i].follow(lead);
        }
    }

    public void setPower(double power) { // -1 <= power <= 1
        lead.set(ControlMode.PercentOutput, power);
    }

    public void setRelVelocity(double vel, SimpleMotorFeedforward aff) { // -1 <= vel <= 1
        setVelocity(vel * Constants.maxVelocity, aff);
    }

    public double getVelocity() {
        return lead.getSelectedSensorVelocity() / Constants.pulsesPerMeter * 10;
    }

    public void resetEncoder() {
        lead.setSelectedSensorPosition(0);
    }

    public void setVelocity(double vel, SimpleMotorFeedforward aff) {// M/S
        double speed = (vel * Constants.pulsesPerMeter / 10.);
        double a = vel - getVelocity();
        double af = aff.calculate(vel, a);
        lead.set(ControlMode.Velocity, speed, DemandType.ArbitraryFeedForward, af);
    }

    public void setVelocity(double vel, double aff) {
        lead.set(ControlMode.Velocity, vel * Constants.pulsesPerMeter / 10.,
                DemandType.ArbitraryFeedForward, aff);
    }

    public double getAccelForSpeed(double vel) {
        return vel - getVelocity();
    }

    public void setK_P(double k_p) {
        lead.config_kP(0, k_p);
    }

    public void setK_I(double k_i) {
        lead.config_kI(0, k_i);
    }

    public void setK_D(double k_d) {
        lead.config_kD(0, k_d);
    }

    public void invertMotors() {
        lead.setInverted(true);
        for (TalonSRX talon : followers) {
            talon.setInverted(InvertType.FollowMaster);
        }
    }

    public double getEncoder() { // Pulses
        return lead.getSelectedSensorPosition();
    }

    public double getDistance() { // Meters
        return getEncoder() / Constants.pulsesPerMeter;
    }

    public void invertEncoder() {
        lead.setSensorPhase(true);
    }
}
