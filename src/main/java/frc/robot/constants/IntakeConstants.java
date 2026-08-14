package frc.robot.constants;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.minolib.hardware.MinoCANDevice;
import frc.robot.Constants;

public class IntakeConstants {
    public class Slam {
        public static final MinoCANDevice kDevice = new MinoCANDevice(14, Constants.kRioBus);

        public static final double kP = 10.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.0;
        public static final double kV = 1.5;
        public static final double kG = 0.0;
        public static final double kA = 0.0;

        public static final boolean kMotorInverted = false;
        public static final double kMotorReduction = (25.0 / 1) * (32.0 / 16.0);
        public static final double kMotorStatorCurrentLimit = 110;
        public static final double kMotorSupplyCurrentLimit = 55;
        public static final DCMotor kSimulatedGearbox = DCMotor.getKrakenX60Foc(1);
    }

    public class Roller {
        public static final MinoCANDevice kDevice = new MinoCANDevice(15, Constants.kRioBus);

        public static final double kP = 10.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.0;
        public static final double kV = 1.5;
        public static final double kA = 0.0;

        public static final boolean kMotorInverted = true;
        public static final double kMotorReduction = (24.0 / 12.0);
        public static final double kMotorStatorCurrentLimit = 120;
        public static final double kMotorSupplyCurrentLimit = 60;
        public static final DCMotor kSimulatedGearbox = DCMotor.getFalcon500Foc(1);
    }
}
