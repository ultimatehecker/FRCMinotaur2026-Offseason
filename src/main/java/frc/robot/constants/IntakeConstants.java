package frc.robot.constants;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

import frc.minolib.advantagekit.LoggedTunableNumber;
import frc.minolib.hardware.MinoCANDevice;
import frc.robot.Constants;

public class IntakeConstants {
    public class Slam {
        public static final MinoCANDevice kKrakenX60 = new MinoCANDevice(14, Constants.kRioBus);
        public static final MinoCANDevice kCANCoder = new MinoCANDevice(14, Constants.kRioBus);

        public static final LoggedTunableNumber kP = new LoggedTunableNumber("Intake/Slam/kP", 10.0);
        public static final LoggedTunableNumber kD = new LoggedTunableNumber("Intake/Slam/kD", 0.0);
        public static final LoggedTunableNumber kS = new LoggedTunableNumber("Intake/Slam/kS", 0.0);
        public static final LoggedTunableNumber kG = new LoggedTunableNumber("Intake/Slam/kG", 0.0);
        public static final LoggedTunableNumber kV = new LoggedTunableNumber("Intake/Slam/kV", 0.0);
        public static final LoggedTunableNumber kA = new LoggedTunableNumber("Intake/Slam/kA", 0.0);

        public static final boolean kMotorInverted = false;
        public static final double kMotorReduction = (25.0 / 1) * (32.0 / 16.0);
        public static final double kMotorStatorCurrentLimit = 110;
        public static final double kMotorSupplyCurrentLimit = 55;
        public static final DCMotor kSimulatedGearbox = DCMotor.getKrakenX60Foc(1);

        public static final double kAbsoluteEncoderOffsetRadians = -2.4908;
    }

    public class Roller {
        public static final MinoCANDevice kFalcon500 = new MinoCANDevice(15, Constants.kRioBus);

        public static final LoggedTunableNumber kP = new LoggedTunableNumber("Intake/Roller/kP", 10.0);
        public static final LoggedTunableNumber kD = new LoggedTunableNumber("Intake/Roller/kD", 0.0);
        public static final LoggedTunableNumber kS = new LoggedTunableNumber("Intake/Roller/kS", 0.0);
        public static final LoggedTunableNumber kV = new LoggedTunableNumber("Intake/Roller/kV", 0.0);
        public static final LoggedTunableNumber kA = new LoggedTunableNumber("Intake/Roller/kA", 0.0);

        public static final boolean kMotorInverted = true;
        public static final double kMotorReduction = (24.0 / 12.0);
        public static final double kMotorStatorCurrentLimit = 120;
        public static final double kMotorSupplyCurrentLimit = 60;
        public static final DCMotor kSimulatedGearbox = DCMotor.getFalcon500Foc(1);

        public static final double kMOI = 0.5 * (0.5) * (Math.pow(Units.inchesToMeters(1.875), 2) + Math.pow(Units.inchesToMeters(2), 2));
    }

    public static final double kLength = Units.inchesToMeters(14.0);
    public static final double kMassKilograms = 5.89;
    public static final double kMOI = 0.02; // Will calculate later

    public static final double kMinimumPosition = Units.degreesToRadians(59.6);
    public static final double kMaximumPosition = Units.degreesToRadians(187.4);

    public static final LoggedTunableNumber kIntakeVoltage = new LoggedTunableNumber("Intake/Roller/IntakeVolts", 13.0);
    public static final LoggedTunableNumber kOuttakeVoltage = new LoggedTunableNumber("Intake/Roller/OuttakeVolts", -6.0);
}
