package frc.robot.constants;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.minolib.advantagekit.LoggedTunableNumber;
import frc.minolib.hardware.MinoCANDevice;
import frc.robot.Constants;

public class FloorConstants {
    public static LoggedTunableNumber kShootingVoltage = new LoggedTunableNumber("Floor/FeedingVoltage", 12.0);
    public static LoggedTunableNumber kFeedingVoltage = new LoggedTunableNumber("Floor/FeedingVoltage", 8.0);
    public static LoggedTunableNumber kExhaustVoltage = new LoggedTunableNumber("Floor/ExhaustVoltage", -8.0);

    public static final MinoCANDevice kLeftKrakenX44 = new MinoCANDevice(16, Constants.kRioBus);
    public static final MinoCANDevice kRightKrakenX44 = new MinoCANDevice(17, Constants.kRioBus);

    public static final LoggedTunableNumber kP = new LoggedTunableNumber("Floor/kP", 10.0);
    public static final LoggedTunableNumber kD = new LoggedTunableNumber("Floor/kD", 0.0);
    public static final LoggedTunableNumber kS = new LoggedTunableNumber("Floor/kS", 0.0);
    public static final LoggedTunableNumber kV = new LoggedTunableNumber("Floor/kV", 0.0);
    public static final LoggedTunableNumber kA = new LoggedTunableNumber("Floor/kA", 0.0);

    public static final boolean kMotorInverted = true;
    public static final double kMotorReduction = (24.0 / 12.0);
    public static final double kMotorStatorCurrentLimit = 120;
    public static final double kMotorSupplyCurrentLimit = 60;
    public static final DCMotor kSimulatedGearbox = DCMotor.getFalcon500Foc(1);

    public static final double kMOI = 0.5 * (0.5) * (Math.pow(Units.inchesToMeters(1.875), 2) + Math.pow(Units.inchesToMeters(2), 2));
}
