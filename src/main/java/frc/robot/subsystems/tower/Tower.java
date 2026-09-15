package frc.robot.subsystems.tower;

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
import frc.robot.constants.TowerConstants;
import frc.robot.subsystems.rollers.RollerSystem;
import frc.robot.subsystems.rollers.RollerSystemIO;

import lombok.Setter;

public class Tower extends SubsystemBase {
    private final RollerSystem tRollerSystem;
    private final RollerSystem bRollerSystem;

    @Setter private BooleanSupplier coastOverride = () -> false;

    private final SysIdRoutine topSysIdRoutine;
    private final SysIdRoutine bottomSysIdRoutine;

    public Tower(RollerSystemIO topIO, RollerSystemIO bottomIO) {
        tRollerSystem = new RollerSystem("Tower/Top", "Tower/Top", topIO);
        bRollerSystem = new RollerSystem("Tower/Bottom", "Tower/Bottom", bottomIO);

        tRollerSystem.setCoastOverride(coastOverride);
        bRollerSystem.setCoastOverride(coastOverride);

        topSysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                state -> Logger.recordOutput("SysId/Tower/Top/State", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                voltage -> tRollerSystem.runOpenLoop(voltage.in(Volts)),
                null,
                this,
                "Tower_Top"
            )
        );

        bottomSysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                state -> Logger.recordOutput("SysId/Tower/Bottom/State", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                voltage -> bRollerSystem.runOpenLoop(voltage.in(Volts)),
                null,
                this,
                "Tower_Bottom"
            )
        );
    }

    @Override
    public void periodic() {
        tRollerSystem.periodic();
        bRollerSystem.periodic();

        if (TowerConstants.topkP.hasChanged(hashCode()) || TowerConstants.topkD.hasChanged(hashCode())) {
            tRollerSystem.setGains(TowerConstants.topkP.get(), TowerConstants.topkD.get());
        }

        if (TowerConstants.topkS.hasChanged(hashCode()) || TowerConstants.topkV.hasChanged(hashCode())) {
            tRollerSystem.setFeedforward(TowerConstants.topkS.get(), TowerConstants.topkV.get());
        }

        if (TowerConstants.bottomkP.hasChanged(hashCode()) || TowerConstants.bottomkD.hasChanged(hashCode())) {
            bRollerSystem.setGains(TowerConstants.bottomkP.get(), TowerConstants.bottomkD.get());
        }

        if (TowerConstants.bottomkS.hasChanged(hashCode()) || TowerConstants.bottomkV.hasChanged(hashCode())) {
            bRollerSystem.setFeedforward(TowerConstants.bottomkS.get(), TowerConstants.bottomkV.get());
        }

        LoggedTracer.record("TowerPeriodic");
    }

    public Command setVelocitySetpoint(DoubleSupplier setpoint) {
        return Commands.runEnd(
            () ->  {
                tRollerSystem.runClosedLoop(setpoint.getAsDouble());
                bRollerSystem.runClosedLoop(setpoint.getAsDouble());
            },
            () -> {
                tRollerSystem.runClosedLoop(0.0);
                bRollerSystem.runClosedLoop(0.0);
            },
            this
        );
    }

    public Command sysIdTopQuasistatic(Direction direction) {
        return topSysIdRoutine.quasistatic(direction);
    }

    public Command sysIdTopDynamic(Direction direction) {
        return topSysIdRoutine.dynamic(direction);
    }

    public Command sysIdBottomDynamic(Direction direction) {
        return bottomSysIdRoutine.quasistatic(direction);
    }

    public Command sysIdRollerDynamic(Direction direction) {
        return bottomSysIdRoutine.dynamic(direction);
    }
}