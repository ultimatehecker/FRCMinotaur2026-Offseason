package frc.robot.subsystems.floor;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.minolib.advantagekit.LoggedTracer;
import frc.robot.subsystems.rollers.RollerSystem;
import frc.robot.subsystems.rollers.RollerSystemIO;

import lombok.Setter;

public class Floor extends SubsystemBase {
    private final RollerSystem rollerSystem;

    @Setter private BooleanSupplier coastOverride = () -> false;

    public Floor(RollerSystemIO io) {
        rollerSystem = new RollerSystem("Floor", "Floor", io);
        rollerSystem.setCoastOverride(coastOverride);
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
}