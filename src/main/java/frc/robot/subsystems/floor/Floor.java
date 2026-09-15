package frc.robot.subsystems.floor;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.minolib.advantagekit.LoggedTracer;
import frc.robot.subsystems.rollers.RollerSystem;
import frc.robot.subsystems.rollers.RollerSystemIO;

import lombok.Setter;

public class Floor extends SubsystemBase {
    private final RollerSystem rollerSystem;

    @Setter private BooleanSupplier coastOverride = () -> false;

    private final SysIdRoutine sysIdRoutine;

    public Floor(RollerSystemIO io) {
        rollerSystem = new RollerSystem("Floor", "Floor", io);
        rollerSystem.setCoastOverride(coastOverride);

        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                state -> Logger.recordOutput("SysId/Floor/State", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                voltage -> rollerSystem.runOpenLoop(voltage.in(Volts)),
                null,
                this,
                "Floor"
            )
        );
    }

    @Override
    public void periodic() {
        rollerSystem.periodic();
        LoggedTracer.record("FloorPeriodic");
    }

    public Command setOpenLoopSetpoint(DoubleSupplier setpoint) {
        return Commands.runEnd(
            () -> rollerSystem.runOpenLoop(setpoint.getAsDouble()),
            () -> rollerSystem.runOpenLoop(0.0),
            this
        );
    }

    public Command sysIdQuasistatic(Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }
}