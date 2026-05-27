package frc.robot.subsystems.intake;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.minolib.advantagekit.LoggedTracer;
import frc.robot.subsystems.intake.slam.Slam;
import frc.robot.subsystems.intake.slam.SlamIO;
import frc.robot.subsystems.rollers.RollerSystem;
import frc.robot.subsystems.rollers.RollerSystemIO;

import lombok.Getter;
import lombok.Setter;

public class Intake extends SubsystemBase {
    private final RollerSystem roller;
    private final Slam slam;

    @AutoLogOutput(key = "Intake/CoastOverride") private BooleanSupplier coastOverride = () -> false;
    @AutoLogOutput(key = "Intake/DisabledOverride") private BooleanSupplier disabledOverride = () -> false;

    @Getter @Setter private IntakeSetpoint setpoint = IntakeSetpoint.idle();

    public Intake(SlamIO slamIO, RollerSystemIO rollerIO) {
        slam = new Slam(slamIO);
        roller = new RollerSystem("Intake Roller", "Intake/Roller", rollerIO);
    }

    @Override
    public void periodic() {
        slam.periodic();
        roller.periodic();

        setBrakeMode(!coastOverride.getAsBoolean());
        applySetpoint(setpoint);

        Logger.recordOutput("Intake/Setpoint/CommandedSetpoint", setpoint.toString());
        Logger.recordOutput("Intake/Setpoint/SlamAngleDegrees", setpoint.slamAngleDegrees());
        Logger.recordOutput("Intake/Setpoint/SlamVelocityRadiansPerSecond", setpoint.slamVelocityRadiansPerSecond());
        Logger.recordOutput("Intake/Setpoint/RollerVoltageSetpoint", setpoint.rollerVoltageSetpoint());
        Logger.recordOutput("Intake/Setpoint/RollerVelocitySetpointRadiansPerSecond", setpoint.rollerVelocityRadiansPerSecond());

        LoggedTracer.record("IntakePeriodic");
    }

    private void applySetpoint(IntakeSetpoint setpoint) {
        if(disabledOverride.getAsBoolean()) {
            slam.stop();
            roller.stop();

            return;
        };

        slam.runSetpoint(setpoint.slamAngleDegrees(), setpoint.slamVelocityRadiansPerSecond());
        roller.setVoltage(setpoint.rollerVoltageSetpoint());
    }

    private void setBrakeMode(boolean enabled) {
        slam.setBrakeMode(enabled);
        roller.setBrakeMode(enabled);
    }

    public void setOverrides(BooleanSupplier coastOverride, BooleanSupplier disabledOverride) {
        this.coastOverride = coastOverride;
        this.disabledOverride = disabledOverride;
    }

    public double getSlamCurrentPositionDegrees() {
        return slam.getMeasuredAngleDegrees();
    }
}
