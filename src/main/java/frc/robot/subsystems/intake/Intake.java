package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.minolib.advantagekit.LoggedTracer;
import frc.minolib.advantagekit.LoggedTunableNumber;
import frc.minolib.utilities.RobotTime;
import frc.robot.RobotState;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.intake.slam.Slam;
import frc.robot.subsystems.intake.slam.SlamIO;
import frc.robot.subsystems.rollers.RollerSystem;
import frc.robot.subsystems.rollers.RollerSystemIO;

public class Intake extends SubsystemBase {
    private RobotState robotState;

    private final RollerSystem roller;
    private final Slam slam;

    private static final LoggedTunableNumber kRetractMaxVelocityRadiansPerSecond = new LoggedTunableNumber("Intake/Slam/RetractMaxVelocity", 3.0);
    private static final LoggedTunableNumber kRetractSlowVelocityRadiansPerSecond = new LoggedTunableNumber("Intake/Slam/SlowRetractMaxVelocity", 0.88);
    private static final LoggedTunableNumber kSlamGoalDebounceTime = new LoggedTunableNumber("Intake/Slam/DebounceTime", 0.5);
    private Debouncer slamGoalDebouncer = new Debouncer(kSlamGoalDebounceTime.get(), DebounceType.kRising);

    private SlewRateLimiter retractSlewRateLimiter = new SlewRateLimiter(kRetractMaxVelocityRadiansPerSecond.get());

    private final SysIdRoutine rollerSysIdRoutine;
    private final SysIdRoutine slamSysIdRoutine;

    public Intake(RobotState robotState, SlamIO slamIO, RollerSystemIO rollerIO) {
        this.robotState = robotState;
        this.slam = new Slam(slamIO);
        this.roller = new RollerSystem("Intake Roller", "Intake/Roller", rollerIO);

        rollerSysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                state -> Logger.recordOutput("SysId/Intake/Roller/State", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                voltage -> roller.runOpenLoop(voltage.in(Volts)),
                null,
                this,
                "Intake_Roller"
            )
        );

        slamSysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.5).per(Second),
                Volts.of(3.0),
                Seconds.of(4.0),
                state -> Logger.recordOutput("SysId/Intake/Slam/State", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                voltage -> slam.runVoltage(voltage.in(Volts)),
                null,
                this,
                "Intake_Slam"
            )
        );
    }

    public void periodic() {
        slam.periodic();
        roller.periodic();

        if (DriverStation.isEnabled()) {
            if (slam.isZeroed()) {
                robotState.addIntakeMotionMeasurements(RobotTime.getTimestampSeconds(), new Rotation2d(slam.getMeasuredAngleRadians()), slam.getVelocity(), roller.getVelocity());
            }
        }

        robotState.addIntakeMotionMeasurements(RobotTime.getTimestampSeconds(), Rotation2d.fromRadians(slam.getMeasuredAngleRadians()), slam.getVelocity(), roller.getVelocity());
        LoggedTracer.record("IntakePeriodic");
    }

    public Command intake() {
        return deploy().andThen(
            Commands.runEnd(
                () -> roller.runOpenLoop(IntakeConstants.kIntakeVoltage.get()),
                () -> roller.runOpenLoop(0.0),
                this
            )
        );
    }

    public Command outtake() {
        return deploy().andThen(
            Commands.runEnd(
                () -> roller.runOpenLoop(IntakeConstants.kOuttakeVoltage.get()),
                () -> roller.runOpenLoop(0.0),
                this
            )
        );
    }

    public Command retract() {
        return retract(kRetractMaxVelocityRadiansPerSecond::get,
            "Intake Retract"
        );
    }

    public Command trashCompact() {
        return Commands.runOnce(() -> roller.runOpenLoop(IntakeConstants.kIntakeVoltage.get() / 2.0)).andThen(retract(
            kRetractSlowVelocityRadiansPerSecond::get,
            "Intake Retract Slow"
        )).finallyDo(() -> roller.runOpenLoop(0.0));
    }

    private Command deploy() {
        return Commands.run(() -> slam.runPosition(IntakeConstants.kMaximumPosition, true), this).until(
            () -> slamGoalDebouncer.calculate(slam.atGoal())
        );
    }

    private Command retract(DoubleSupplier maxVelocitySupplier, String name) {
        return Commands.defer(() -> {
            retractSlewRateLimiter = new SlewRateLimiter(maxVelocitySupplier.getAsDouble());
            retractSlewRateLimiter.reset(slam.getMeasuredAngleRadians());

            return Commands.run(() -> {
                double setpoint = retractSlewRateLimiter.calculate(IntakeConstants.kMaximumPosition);
                slam.runPosition(setpoint, true);
            }, this);
        }, Set.of(this)).withName(name);
    }

    public Command sysIdRollerQuasistatic(Direction direction) {
        return rollerSysIdRoutine.quasistatic(direction);
    }

    public Command sysIdRollerDynamic(Direction direction) {
        return rollerSysIdRoutine.dynamic(direction);
    }

    public Command sysIdSlamQuasistatic(Direction direction) {
        return slamSysIdRoutine.quasistatic(direction).until(() -> reachedSysIdLimit(direction));
    }

    public Command sysIdSlamDynamic(Direction direction) {
        return slamSysIdRoutine.dynamic(direction).until(() -> reachedSysIdLimit(direction));
    }

    public void setCoastOverride(BooleanSupplier coast) {
        slam.setCoastOverride(coast);
        roller.setCoastOverride(coast);
    }

    public boolean isZeroed() {
        return slam.isZeroed();
    }

    private boolean reachedSysIdLimit(Direction direction) {
        double position = slam.getMeasuredAngleRadians();

        return switch (direction) {
            case kForward ->
                position >= IntakeConstants.kMaximumPosition;

            case kReverse ->
                position <= IntakeConstants.kMinimumPosition;
        };
    }

    public Command homeSlam() {
        return slam.zeroCommand().deadlineFor(Commands.idle(this));
    }

    public Command zeroMaxSlam() {
        return Commands.runOnce(() -> slam.zeroMaxAngle()).ignoringDisable(true);
    }
}