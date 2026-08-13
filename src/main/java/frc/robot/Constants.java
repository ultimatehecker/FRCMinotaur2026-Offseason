package frc.robot;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.minolib.hardware.MinoCANBus;

public class Constants {
    public static final int kLoopBackTimeSeconds = 1;
    public static final double kLoopPeriodSeconds = 0.02;
    public static final double kSimLoopPeriodSeconds = 0.005;

    public static final MinoCANBus kRioBus = new MinoCANBus("rio");
    public static final MinoCANBus kCANivoreBus = new MinoCANBus("*");

    public static final double kLowBatteryVoltage = 10.0;
    public static final double kLowBatteryDisabledTime = 1.5;
    
    public static final double kRioCANErrorTimeThreshold = 0.5;
    public static final double kCANivoreErrorTimeThreshold = 0.5;

    private static RobotType kRobotType = RobotType.SIMBOT;
    public static final boolean kTuningMode = true;
    public static final boolean kUseMapleSim = true;

    @SuppressWarnings("resource")
    public static RobotType getRobot() {
        if (!disableHAL && RobotBase.isReal() && kRobotType == RobotType.SIMBOT) {
            new Alert("Invalid robot selected, using competition robot as default.", AlertType.kInfo).set(true);
            kRobotType = RobotType.COMPBOT;
        }

        return kRobotType;
    }

    public static Mode getMode() {
        return switch (kRobotType) {
            case DEVBOT, COMPBOT -> RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
            case SIMBOT -> Mode.SIM;
        };
    }

    public enum Mode {
        REAL,
        SIM,
        REPLAY
    }

    public enum RobotType {
        SIMBOT,
        DEVBOT,
        COMPBOT
    }

    public static boolean disableHAL = false;

    public static void disableHAL() {
        disableHAL = true;
    }

    /** Checks whether the correct robot is selected when deploying. */
    public static class CheckDeploy {
        public static void main(String... args) {
            if (kRobotType == RobotType.SIMBOT) {
                System.err.println("Cannot deploy, invalid robot selected: " + kRobotType);
                System.exit(1);
            }
        }
    }
}