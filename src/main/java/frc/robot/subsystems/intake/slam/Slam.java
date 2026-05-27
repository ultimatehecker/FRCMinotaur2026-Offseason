package frc.robot.subsystems.intake.slam;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.minolib.advantagekit.LoggedTracer;
import frc.minolib.advantagekit.LoggedTunableNumber;
import frc.minolib.energy.EnergyManagement;
import frc.minolib.math.EqualsUtility;
import frc.robot.Robot;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.intake.IntakeSetpoint;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.BooleanSupplier;

import lombok.Getter;

public class Slam {
    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Intake/Slam/Gains/kP");
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Intake/Slam/Gains/kD");
    private static final LoggedTunableNumber kS = new LoggedTunableNumber("Intake/Slam/Gains/kS");
    private static final LoggedTunableNumber kV = new LoggedTunableNumber("Intake/Slam/Gains/kV");
    private static final LoggedTunableNumber kG = new LoggedTunableNumber("Intake/Slam/Gains/kG");
    private static final LoggedTunableNumber kA = new LoggedTunableNumber("Intake/Slam/Gains/kA");

    private static final LoggedTunableNumber kMinimumAngleDegrees = new LoggedTunableNumber("Intake/Slam/MinimumAngleDegrees", IntakeConstants.kIntakeMinimumPosition.in(Degrees));
    private static final LoggedTunableNumber kMaximumAngleDegrees = new LoggedTunableNumber("Intake/Slam/MaximumAngleDegrees", IntakeConstants.kIntakeMaximumPosition.in(Degrees));

    private static final LoggedTunableNumber kHomingVoltage = new LoggedTunableNumber("Intake/Slam/Homing/Voltage", -3);
    private static final LoggedTunableNumber kHomingVelocityThreshold = new LoggedTunableNumber("Intake/Slam/Homing/VelocityThreshold", 0.06);
    private static final LoggedTunableNumber kHomingTimeoutSeconds = new LoggedTunableNumber("Intake/Slam/Homing/TimeoutSeconds", 0.2);

    private static final LoggedTunableNumber kToleranceDegrees = new LoggedTunableNumber("Intake/Slam/ToleranceDegrees", 1.0);
    private static final LoggedTunableNumber kReadyDebounceSeconds = new LoggedTunableNumber("Intake/Slam/ReadyDebounceSeconds", 0.08);

    static {
        switch (GlobalConstants.getRobot()) {
            default -> {
                kP.initDefault(IntakeConstants.pivotkP);
                kD.initDefault(IntakeConstants.pivotkD);
                kS.initDefault(IntakeConstants.pivotkS);
                kV.initDefault(IntakeConstants.pivotkV);
                kG.initDefault(IntakeConstants.pivotkG);
                kA.initDefault(IntakeConstants.pivotkA);
            }
            case SIMBOT -> {
                kP.initDefault(10.0);
                kD.initDefault(0.0);
                kS.initDefault(0.0);
                kV.initDefault(0.0);
                kG.initDefault(0.0);
                kA.initDefault(0.0);
            }
        }
    }

    private final SlamIO io;
    private final SlamIOInputsAutoLogged inputs = new SlamIOInputsAutoLogged();

    private final Debouncer motorConnectedDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private Debouncer homingDebouncer = new Debouncer(kHomingTimeoutSeconds.get(), Debouncer.DebounceType.kRising);
    private Debouncer readyDebouncer = new Debouncer(kReadyDebounceSeconds.get(), Debouncer.DebounceType.kFalling);

    private final Alert motorDisconnectedAlert = new Alert("Slam motor disconnected!", Alert.AlertType.kError);
    private final Alert motorTemperatureAlert = new Alert("Slam motor is overheating!", Alert.AlertType.kWarning);

    @AutoLogOutput(key = "Intake/Slam/BrakeModeEnabled") 
    private boolean brakeModeEnabled = false;

    @Getter private boolean zeroed = true;
    private double targetAngleRadians = Double.NaN;
    private double targetVelocityRadiansPerSecond = IntakeSetpoint.getFastMaxVelocityRadiansPerSecond();

    private final Command zeroCommand;

    public Slam(SlamIO io) {
        this.io = io;

        zeroCommand = zeroCommand();
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake/Slam", inputs);

        motorDisconnectedAlert.set(!motorConnectedDebouncer.calculate(inputs.isMotorConnected) && !Robot.isJITing());
        motorTemperatureAlert.set(inputs.temperatureFault);

        EnergyManagement.getInstance().reportCurrentUsage("Slam", false, inputs.isMotorConnected ? inputs.supplyCurrentAmperes : 0.0);

        // Update tunable numbers
        if (kP.hasChanged(hashCode()) || kD.hasChanged(hashCode()) || kS.hasChanged(hashCode()) || kV.hasChanged(hashCode()) || kG.hasChanged(hashCode()) || kA.hasChanged(hashCode())) {
            io.setPID(kP.get(), 0.0, kD.get(), kS.get(), kV.get(), kG.get(), kA.get());
        }

        if (kReadyDebounceSeconds.hasChanged(hashCode()) || kHomingTimeoutSeconds.hasChanged(hashCode())) {
            readyDebouncer.setDebounceTime(kReadyDebounceSeconds.get());
            homingDebouncer.setDebounceTime(kHomingTimeoutSeconds.get());
        }

        if (DriverStation.isEnabled() && !zeroed && !zeroCommand.isScheduled()) {
            CommandScheduler.getInstance().schedule(zeroCommand);
        }

        if(targetAngleRadians != Double.NaN) io.setPosition(targetAngleRadians, targetVelocityRadiansPerSecond);

        Logger.recordOutput("Intake/Slam/Zeroed", zeroed);
        Logger.recordOutput("Intake/Slam/TargetAngleDegrees", Units.radiansToDegrees(targetAngleRadians));
        SmartDashboard.putBoolean("Is Slam Retracted?", EqualsUtility.epsilonEquals(inputs.positionRadians, Units.degreesToRadians(kMinimumAngleDegrees.get())));
        
        LoggedTracer.record("SlamPeriodic");
    }

    @AutoLogOutput(key = "Intake/Slam/MeasuredAngleDegrees")
    public double getMeasuredAngleDegrees() {
        return Units.radiansToDegrees(inputs.positionRadians);
    }

    @AutoLogOutput(key = "Intake/Slam/MeasuredVelocityDegreesPerSecond")
    public double getMeasuredVelocityDegreesPerSecond() {
        return Units.radiansToDegrees(inputs.velocityRadiansPerSecond);
    }


    public void runSetpoint(double degrees, double degreesPerSecond) {
        targetAngleRadians = Units.degreesToRadians(degrees);
        targetVelocityRadiansPerSecond = Units.degreesToRadians(degreesPerSecond);
    }

    public void setBrakeMode(boolean enabled) {
        if (brakeModeEnabled == enabled) return;
        brakeModeEnabled = enabled;
        io.setBrakeMode(brakeModeEnabled);
    }

    public void stow() {
        targetAngleRadians = Units.degreesToRadians(kMinimumAngleDegrees.get());
        targetVelocityRadiansPerSecond = IntakeSetpoint.getFastMaxVelocityRadiansPerSecond();
    }

    public void stop() {
        targetAngleRadians = Double.NaN;
        readyDebouncer.calculate(false);
        io.stop();
    }

    @AutoLogOutput(key = "Intake/Slam/AtTarget")
    public boolean atTarget() {
        if (Double.isNaN(targetAngleRadians)) return false;
        return Math.abs(inputs.positionRadians - targetAngleRadians) <= Units.degreesToRadians(kToleranceDegrees.get());
    }

    @AutoLogOutput(key = "Intake/Slam/IsReady")
    public boolean isReady() {
        return zeroed && readyDebouncer.calculate(atTarget());
    }

    public Command stowCommand() {
        return Commands.run(this::stow).withName("Stow Slam");
    }

    public Command zeroCommand() {
        return Commands.sequence(
            Commands.runOnce(() -> {
                zeroed = false;
                homingDebouncer = new Debouncer(kHomingTimeoutSeconds.get(), Debouncer.DebounceType.kRising);
                homingDebouncer.calculate(false);
                targetAngleRadians = Double.NaN;
            }),
            Commands.run(() -> {
                io.setVoltage(kHomingVoltage.get());
            }).until(() -> {
                    return homingDebouncer.calculate(Math.abs(inputs.velocityRadiansPerSecond) <= kHomingVelocityThreshold.get() && Math.abs(inputs.appliedVoltage) >= Math.abs(kHomingVoltage.get()) * 0.7);
            }).withTimeout(3.0),
            Commands.runOnce(() -> {
                io.resetPosition();

                zeroed = true;
                stow();
            })
        )
        .withName("Zero Slam")
        .ignoringDisable(false);
    }
}