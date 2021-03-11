/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package frc.robot.utils;

import frc.robot.Constants;

/**
 *
 * @author Udi Kislev
 */
public class FeedForward {
    
    public static double K_HA = 6;
    public static double K_LA = 0.25;
    public static double K_S = Constants.CHASSIS_KS / 12 * 0.97; //0.22404
    public static double K_V = Constants.CHASSIS_KV / 12 * 1.137; //0.04314
    public static double MAX_BRAKEMODE = 0.05;
    public static double MIN_BRAKEMODE = 0.1;
    
    public static double feedForwardPower(double velocity) {
        return velocity * K_V + K_S;
    }
    
    public static double feedForwardLeftPower(double leftVelocity, double rightVelocity) {
        // if zeros - return 0
        if(leftVelocity == 0 && rightVelocity == 0) {
            return 0;
        }
        // if not in same direction - use normal power
        if(leftVelocity * rightVelocity < 0) {
            return feedForwardPower(leftVelocity);
        }
        // if negative - return minus of the positive values
        if(leftVelocity < 0) {
            return -feedForwardLeftPower(-leftVelocity, -rightVelocity);
        }
        // if very small velocity and other side is higher - retuen 0 for brake mode
        if(leftVelocity < MAX_BRAKEMODE && rightVelocity > MIN_BRAKEMODE) {
            return 0;
        }
        if(leftVelocity > rightVelocity) {
            return highPower(leftVelocity, rightVelocity);
        } else {
            return lowPower(rightVelocity, leftVelocity);
        }
    }

    public static double feedForwardRightPower(double leftVelocity, double rightVelocity) {
        return feedForwardLeftPower(rightVelocity, leftVelocity);
    }
    
    private static double power(double highVelocity, double lowVelocity, boolean high) {
        double v = (highVelocity - lowVelocity + highVelocity*K_HA - lowVelocity*K_LA) / (K_HA - K_LA);
        double p = feedForwardPower(v);
        if(high) {
            return p;
        } else {
            return p - (v-lowVelocity) * K_LA;
        }
    }
    private static double highPower(double highVelocity, double lowVelocity) {
        return power(highVelocity, lowVelocity, true);
    }
    private static double lowPower(double highVelocity, double lowVelocity) {
        return power(highVelocity, lowVelocity, false);
    }
}
