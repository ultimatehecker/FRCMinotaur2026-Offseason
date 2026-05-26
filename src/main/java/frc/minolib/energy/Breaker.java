package frc.minolib.energy;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

import frc.minolib.math.MathUtility;
import frc.robot.constants.GlobalConstants;
import lombok.Getter;

public class Breaker {
    public static final double I_RATED = 120.0;
    private static final double TAU_COOL = 60.0; // Cooldown time constant

    // Maximum hold times from breaker datasheet
    private static final double[] I_NORM_PTS = {1.35, 2.0, 2.25, 2.5, 3.0, 4.0, 5.0};
    private static final double[] TRIP_TIME_PTS = {30.0 * 60.0, 70.0, 38.0, 25.0, 15.0, 10.0, 7.0};
    private static final double kSentinelTripTime = 1.0e6;
    private static final double kMinimumTripTime = TRIP_TIME_PTS[TRIP_TIME_PTS.length - 1];

    private static final InterpolatingDoubleTreeMap logTripTimeMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap logInverseTripTimeMap = new InterpolatingDoubleTreeMap();

    static {
        logTripTimeMap.put(Math.log(1.0), Math.log(kSentinelTripTime));
        logInverseTripTimeMap.put(Math.log(kSentinelTripTime), Math.log(1.0));

        for (int i = 0; i < I_NORM_PTS.length; i++) {
            logTripTimeMap.put(Math.log(I_NORM_PTS[i]), Math.log(TRIP_TIME_PTS[i]));
            logInverseTripTimeMap.put(Math.log(TRIP_TIME_PTS[i]), Math.log(I_NORM_PTS[i]));
        }
    }

    private final double tripThreshold;
    @Getter private double damageState = 0.0;

    public Breaker(double niceness) {
        tripThreshold = 1.0 - MathUtility.clamp(niceness, 0.0, 1.0);
    }

    public double calculateMaxCurrent(double budgetPeriodSecs) {
        double remaining = tripThreshold - damageState;

        if (remaining <= 0.0) {
            return 0.0;
        }

        double requiredTripTime = budgetPeriodSecs / remaining;
        if (requiredTripTime >= kSentinelTripTime) {
            return Double.MAX_VALUE;
        }

        if (requiredTripTime <= kMinimumTripTime) {
            return I_NORM_PTS[I_NORM_PTS.length - 1] * I_RATED;
        }

        return Math.exp(logInverseTripTimeMap.get(Math.log(requiredTripTime))) * I_RATED;
    }

    public void update(double current) {
        double dt = GlobalConstants.kLoopPeriodSeconds;
        double normalizedI = current / I_RATED;

        boolean cooling;
        if (normalizedI > 1.0) {
            damageState += dt / getTripTime(normalizedI);
            cooling = false;
        } else {
            damageState *= Math.exp(-dt / TAU_COOL);
            cooling = true;
        }

        damageState = MathUtility.clamp(damageState, 0.0, 1.0);

        Logger.recordOutput("Breaker/Cooling", cooling);
        Logger.recordOutput("Breaker/DamageState", damageState);
    }

    /** Returns the interpolated trip time for the given normalized current. */
    public static double getTripTime(double normalizedI) {
        return Math.exp(logTripTimeMap.get(Math.log(Math.max(normalizedI, 1.0))));
    }
}