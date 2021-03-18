/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils;

import java.util.Arrays;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import frc.robot.Constants;

public class GroupOfMotors {
    private WPI_TalonFX lead;
    private WPI_TalonFX[] followers;

    /**
     * Recieves a group of talon device numbers. Creats an instance of WPI_TalonFX
     * talons for each device number.
     * sets the first to be the leader and the rest to follow it.
     * also configs the characteristics to those in the constants and sets them to
     * brake mode.
     * 
     * @param ports - a group of device numbers for WPI_TalonFX talons
     */
    public GroupOfMotors(int... ports) {
        this(Arrays.stream(ports).mapToObj(port -> new WPI_TalonFX(port))
                .toArray(WPI_TalonFX[]::new));
    }

    /**
     * Recieves a group of talons. sets the first to be the leader and the rest to
     * follow it.
     * also configs the characteristics to those in the constants and sets them to
     * brake mode.
     * 
     * @param talons - a group of WPI_TalonFX talons
     */
    public GroupOfMotors(WPI_TalonFX... talons) {
        this.lead = talons[0];
        this.lead.config_kP(0, Constants.CHASSIS_KP);
        this.lead.config_kD(0, Constants.CHASSIS_KD);
        this.lead.setNeutralMode(NeutralMode.Brake);
        followers = new WPI_TalonFX[talons.length - 1];
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
        setVelocity(vel * Constants.MAX_VELOCITY, aff);
    }

    public double getVelocity() {
        return lead.getSelectedSensorVelocity() / Constants.PULSES_PER_METER * 10;
    }

    public void resetEncoder() {
        lead.setSelectedSensorPosition(0);
    }

    public void setVelocity(double vel, SimpleMotorFeedforward aff) {// M/S
        double a = (vel - getVelocity());
        double af = aff.calculate(vel, a) / 12.;
        setVelocity(vel, af);
    }

    public void setVelocity(double vel, double aff) {
        lead.set(ControlMode.Velocity, vel * Constants.PULSES_PER_METER / 10.,
        DemandType.ArbitraryFeedForward, aff);
    }

    public void setPosition(double pos) {
        this.lead.set(ControlMode.Position, pos * Constants.PULSES_PER_METER);
    }

    public void setPosition(double pos, double ff) {
        this.lead.set(ControlMode.Position, pos * Constants.PULSES_PER_METER ,
                DemandType.ArbitraryFeedForward, ff);
    }

    public double getPower() {
        return this.lead.getMotorOutputPercent();         
    }
    /**
     * 
     * @param error - in sensor units
     */
    public void setClosedLoopAllowedError(double error) {
        this.lead.configAllowableClosedloopError(0, 150); 
    }

    public double getError() {
        return this.lead.getClosedLoopError(); 
    }
    public void setMotionMagic(double pos, SimpleMotorFeedforward aff, double maxSpeed,
            double acceleration) {
        // this.lead.set(ControlMode.MotionMagic, pos, DemandType.ArbitraryFeedForward,
        // aff.calculate(maxSpeed, acceleration));
        this.lead.set(ControlMode.MotionMagic, pos, DemandType.Neutral,
                aff.calculate(maxSpeed, acceleration));
    }

    public void setMotionMagic(double pos) {
        // this.lead.set(ControlMode.MotionMagic, pos, DemandType.Neutral, 0);
        this.lead.set(ControlMode.MotionMagic, pos, DemandType.AuxPID, 0);
    }

    public double getAccelForSpeed(double vel) {
        return vel - getVelocity();
    }

    public void setK_P(double k_p) {
        this.lead.config_kP(0, k_p);
    }

    public void setK_I(double k_i) {
        this.lead.config_kI(0, k_i);
    }

    public void setK_D(double k_d) {
        this.lead.config_kD(0, k_d);
    }

    public void invertMotors() {
        this.lead.setInverted(true);
        for (WPI_TalonFX talon : followers) {
            talon.setInverted(InvertType.FollowMaster);
        }
    }

    /**
     * 
     * @return the encoder pulse count
     */
    public double getEncoder() { // Pulses
        return this.lead.getSelectedSensorPosition();
    }

    /**
     * 
     * @return the encoder count translated to meters
     */
    public double getDistance() { // Meters
        return this.getEncoder() / Constants.PULSES_PER_METER;
    }

    public void invertEncoder() {
        this.lead.setSensorPhase(true);
    }

    /**
     * 
     * @param isBrake bake mode or coast
     * Sets the motors neutral mode to either brake or coast
     */
    public void setNeutralMode(boolean isBrake) {
        NeutralMode mode = isBrake ? NeutralMode.Brake : NeutralMode.Coast;
        lead.setNeutralMode(mode);
        for (WPI_TalonFX follower : followers) {
            follower.setNeutralMode(mode);
        }
    }

    /**
     * If the S-Curve strength [0,8] is set to a nonzero value,
     * the generated velocity profile is no longer trapezoidal,
     * but instead is continuous (corner points are smoothed).
     * The S-Curve feature, by its nature, will increase the amount of time a
     * movement requires.
     * This can be compensated for by decreasing the configured acceleration value.
     * 
     * @param curveStrength - the S-Curve strength between 0-8
     */
    public void setMotionSCurve(int curveStrength) {
        this.lead.configMotionSCurveStrength(curveStrength);
    }

    /**
     * configs the talon's cruise (peak) velocity
     * 
     * @param cruiseVelocity - in sensor units per 100ms
     */
    public void setCruiseVelocity(double cruiseVelocity) {
        this.lead.configMotionCruiseVelocity(cruiseVelocity);
    }

    /**
     * configs the motion acceleration
     * 
     * @param motionAcceleration - in sensor units per 100ms
     */
    public void setAcceleration(double motionAcceleration) {
        this.lead.configMotionAcceleration(motionAcceleration);
    }
}
