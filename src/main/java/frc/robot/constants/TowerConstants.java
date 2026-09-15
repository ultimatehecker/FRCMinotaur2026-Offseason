package frc.robot.constants;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.minolib.advantagekit.LoggedTunableNumber;
import frc.minolib.hardware.MinoCANDevice;
import frc.robot.Constants;

public class TowerConstants {
    public static LoggedTunableNumber kShootingVoltage = new LoggedTunableNumber("Tower/FeedingVoltage", 12.0);
    public static LoggedTunableNumber kFeedingVoltage = new LoggedTunableNumber("Tower/FeedingVoltage", 8.0);
    public static LoggedTunableNumber kExhaustVoltage = new LoggedTunableNumber("Tower/ExhaustVoltage", -8.0);

    public static final LoggedTunableNumber topkP = new LoggedTunableNumber("Tower/Top/Gains/kP", 1.5);
    public static final LoggedTunableNumber topkD = new LoggedTunableNumber("Tower/Top/Gains/kD", 0.0);
    public static final LoggedTunableNumber topkS = new LoggedTunableNumber("Tower/Top/Gains/kS", 0.48);
    public static final LoggedTunableNumber topkV = new LoggedTunableNumber("Tower/Top/Gains/kV", 0.056);

    public static final LoggedTunableNumber bottomkP = new LoggedTunableNumber("Tower/Bottom/Gains/kP", 3.0);
    public static final LoggedTunableNumber bottomkD = new LoggedTunableNumber("Tower/Bottom/Gains/kD", 0.0);
    public static final LoggedTunableNumber bottomkS = new LoggedTunableNumber("Tower/Bottom/Gains/kS", 0.4);
    public static final LoggedTunableNumber bottomkV = new LoggedTunableNumber("Tower/Bottom/Gains/kV", 0.0575);

    public static final MinoCANDevice kBottomFalcon500 = new MinoCANDevice(18, Constants.kRioBus);
    public static final MinoCANDevice kTopKrakenX60 = new MinoCANDevice(19, Constants.kRioBus);

    public static final boolean kTopMotorInverted = true;
    public static final double kTopMotorReduction = (24.0 / 12.0);
    public static final double kTopMotorStatorCurrentLimit = 120;
    public static final double kTopMotorSupplyCurrentLimit = 60;
    public static final DCMotor kTopSimulatedGearbox = DCMotor.getKrakenX60Foc(1);

    public static final boolean kBottomMotorInverted = true;
    public static final double kBottomMotorReduction = (24.0 / 12.0);
    public static final double kBottomMotorStatorCurrentLimit = 120;
    public static final double kBottomMotorSupplyCurrentLimit = 60;
    public static final DCMotor kBottomSimulatedGearbox = DCMotor.getFalcon500Foc(1);

    public static final double kMOI = 0.5 * (0.5) * (Math.pow(Units.inchesToMeters(1.875), 2) + Math.pow(Units.inchesToMeters(2), 2));
}
