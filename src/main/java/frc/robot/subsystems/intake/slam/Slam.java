package frc.robot.subsystems.intake.slam;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.minolib.advantagekit.LoggedTunableNumber;
import frc.robot.Robot;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.intake.slam.SlamIO.SlamIOOutputMode;
import frc.robot.subsystems.intake.slam.SlamIO.SlamIOOutputs;

import lombok.Getter;
import lombok.Setter;

public class Slam {
    private final SlamIO io;
    private final SlamIOInputsAutoLogged inputs = new SlamIOInputsAutoLogged();
    private final SlamIOOutputs outputs = new SlamIOOutputs();

    private static final LoggedTunableNumber kToleranceRadians = new LoggedTunableNumber("Intake/Slam/ToleranceRadians", Units.degreesToRadians(4));
    private static final LoggedTunableNumber kHomingVoltage = new LoggedTunableNumber("Intake/Slam/Homing/Volts", -4);
    private static final LoggedTunableNumber kHomingVelocityThreshold = new LoggedTunableNumber("Intake/Slam/Homing/VelocityThreshold", .05);

    private final Debouncer motorConnectedDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private final Alert motorDisconnectedAlert = new Alert("The intake slam motor is disconnected!", Alert.AlertType.kError);
    private final Alert motorOverheatingAlert = new Alert("The intake slam motor is overheating!", Alert.AlertType.kWarning);

    private final Debouncer absoluteEncoderConnectedDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private final Alert absoluteEncoderDisconnectedAlert = new Alert("The intake slam absolute encoder is disconnected!", Alert.AlertType.kError);

    @AutoLogOutput(key = "Intake/Slam/GoalAngle") private double goalAngle = 0.0;
    private static double slamOffset = 0.0;

    @Getter private boolean zeroed = true;

    @Setter private BooleanSupplier coastOverride = () -> false;

    public Slam(SlamIO io) {
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake/Slam", inputs);

        motorDisconnectedAlert.set(Robot.showHardwareAlerts() && !motorConnectedDebouncer.calculate(inputs.motorConnected));
        motorOverheatingAlert.set(inputs.temperatureFault);
        absoluteEncoderDisconnectedAlert.set(Robot.showHardwareAlerts() && !absoluteEncoderConnectedDebouncer.calculate(inputs.absoluteEncoderConnected) && outputs.mode == SlamIOOutputMode.CLOSED_LOOP_SENSORED);

        Robot.batteryLogger.reportCurrentUsage("Intake/Slam", inputs.motorConnected ? inputs.supplyCurrentAmperes : 0.0);

        if (IntakeConstants.Slam.kP.hasChanged(hashCode()) 
            || IntakeConstants.Slam.kD.hasChanged(hashCode()) 
            || IntakeConstants.Slam.kS.hasChanged(hashCode()) 
            || IntakeConstants.Slam.kG.hasChanged(hashCode()) 
            || IntakeConstants.Slam.kV.hasChanged(hashCode()) 
            || IntakeConstants.Slam.kA.hasChanged(hashCode())
        ) {
            outputs.kP = IntakeConstants.Slam.kP.get();
            outputs.kD = IntakeConstants.Slam.kD.get();
            outputs.kS = IntakeConstants.Slam.kS.get();
            outputs.kG = IntakeConstants.Slam.kG.get();
            outputs.kV = IntakeConstants.Slam.kV.get();
            outputs.kA = IntakeConstants.Slam.kA.get();
        }

        if (DriverStation.isDisabled()) {
            outputs.mode = SlamIOOutputMode.BRAKE;

            if (coastOverride.getAsBoolean()) {
                outputs.mode = SlamIOOutputMode.COAST;
            }
        }

        io.applyOutputs(outputs);
    }

    public void runVoltage(double volts) {
        outputs.mode = SlamIOOutputMode.VOLTAGE_CONTROL;
        outputs.appliedVoltage = volts;
    }

    public void runPosition(double positionRadians, boolean sensored) {
        goalAngle = positionRadians;

        SlamIOOutputMode newMode = sensored ? SlamIOOutputMode.CLOSED_LOOP_SENSORED : SlamIOOutputMode.CLOSED_LOOP_UNSENSORED;

        if (newMode == SlamIOOutputMode.CLOSED_LOOP_SENSORED) {
            zeroed = true;
        } else if (outputs.mode == SlamIOOutputMode.CLOSED_LOOP_SENSORED && newMode == SlamIOOutputMode.CLOSED_LOOP_UNSENSORED) {
            zeroed = false;
        }

        outputs.mode = newMode;
        outputs.position = MathUtil.clamp(goalAngle, IntakeConstants.kMinimumPosition, IntakeConstants.kMaximumPosition) - (sensored ? 0.0 : slamOffset);
    }

    @AutoLogOutput(key = "Intake/Slam/MeasuredAngleRadians")
    public double getMeasuredAngleRad() {
        return inputs.positionRadians + slamOffset;
    }

    @AutoLogOutput
    public boolean atGoal() {
        return DriverStation.isEnabled() && zeroed && Math.abs(getMeasuredAngleRad() - goalAngle) <= kToleranceRadians.get();
    }

    private void zero() {
        slamOffset = IntakeConstants.kMinimumPosition - inputs.positionRadians;
        zeroed = true;
    }

    public Command zeroCommand() {
        return Commands.run(() -> {
            runVoltage(kHomingVoltage.get());
            zeroed = false;
        }).raceWith(
            Commands.waitSeconds(1.0).andThen(
                Commands.waitUntil(() -> Math.abs(inputs.velocityRadiansPerSecond) <= kHomingVelocityThreshold.get())
            )
        ).andThen(this::zero);
    }

    public double getVelocity() {
        return inputs.velocityRadiansPerSecond;
    }
}