package frc.robot.subsystems.intake;

import frc.minolib.advantagekit.LoggedTunableNumber;

public record IntakeSetpoint(double slamAngleDegrees, double slamVelocityRadiansPerSecond, double rollerVoltageSetpoint, double rollerVelocityRadiansPerSecond) {
    private static final LoggedTunableNumber kStowedPosition = new LoggedTunableNumber("Intake/Slam/StowedPositionDegrees", 56.9);
    private static final LoggedTunableNumber kDeployedPosition = new LoggedTunableNumber("Intake/Slam/DeployedPositionDegrees", 187.4);
    private static final LoggedTunableNumber kHalfDeployPosition = new LoggedTunableNumber("Intake/Slam/HalfPositionDegrees", 110.0);
    private static final LoggedTunableNumber kFeedPosition = new LoggedTunableNumber("Intake/Slam/FeedPositionDegrees", 140.0);

    private static final LoggedTunableNumber kRetractMaxVelocityRadiansPerSecond = new LoggedTunableNumber("Intake/Slam/RetractMaxVelocityRadiansPerSecond", 3.0);
    private static final LoggedTunableNumber kSlowMaxVelocityRadiansPerSecond = new LoggedTunableNumber("Intake/Slam/SlowRetractMaxVelocityRadiansPerSecond", 0.88);
    private static final LoggedTunableNumber kFastMaxVelocityRadiansPerSecond = new LoggedTunableNumber("Intake/Slam/FastRetractMaxVelocityRadiansPerSecond", 1.3);

    private static final LoggedTunableNumber kIntakeVoltage = new LoggedTunableNumber("Intake/Roller/IntakeVoltage", 7.0);
    private static final LoggedTunableNumber kExhaustVoltage = new LoggedTunableNumber("Intake/Roller/ExhaustVoltage", -6.0);
    private static final LoggedTunableNumber kIdleVoltage = new LoggedTunableNumber("Intake/Roller/IdleVoltage", 3.0);

    private static final LoggedTunableNumber kIntakeVelocityRadiansPerSecond = new LoggedTunableNumber("Intake/Roller/IntakeVelocityRadiansPerSecond", 370.0);
    private static final LoggedTunableNumber kExhaustVelocityRadiansPerSecond = new LoggedTunableNumber("Intake/Roller/ExhaustVelocityRadiansPerSecond", -315.0);
    private static final LoggedTunableNumber kIdleVelocityRadiansPerSecond = new LoggedTunableNumber("Intake/Roller/IdleVelocityRadiansPerSecond", 80.0);

    public static IntakeSetpoint slowRetract() {
        return new IntakeSetpoint(
            kStowedPosition.get(),
            kSlowMaxVelocityRadiansPerSecond.get(),
            kIdleVoltage.get(),
            kIdleVelocityRadiansPerSecond.get());
    }

    public static IntakeSetpoint fastRetract() {
        return new IntakeSetpoint(
            kStowedPosition.get(),
            kFastMaxVelocityRadiansPerSecond.get(),
            0.0,
            0.0);
    }

    public static IntakeSetpoint idle() {
        return new IntakeSetpoint(
            kStowedPosition.get(),
            kSlowMaxVelocityRadiansPerSecond.get(),
            kIdleVoltage.get(),
            kIdleVelocityRadiansPerSecond.get());
    }

    public static IntakeSetpoint deploy() {
        return new IntakeSetpoint(
            kDeployedPosition.get(),
            kFastMaxVelocityRadiansPerSecond.get(),
            0.0,
            0.0);
    }

    public static IntakeSetpoint intake() {
        return new IntakeSetpoint(
            kDeployedPosition.get(),
            kFastMaxVelocityRadiansPerSecond.get(),
            kIntakeVoltage.get(),
            kIntakeVelocityRadiansPerSecond.get());
    }

    public static IntakeSetpoint exhaustDeployed() {
        return new IntakeSetpoint(
            kDeployedPosition.get(),
            kFastMaxVelocityRadiansPerSecond.get(),
            kExhaustVoltage.get(),
            kExhaustVelocityRadiansPerSecond.get());
    }

    public static IntakeSetpoint exhaustHalfDeployed() {
        return new IntakeSetpoint(
            kHalfDeployPosition.get(),
            kFastMaxVelocityRadiansPerSecond.get(),
            kExhaustVoltage.get(),
            kExhaustVelocityRadiansPerSecond.get());
    }

    public static IntakeSetpoint wall() {
        return new IntakeSetpoint(
            kFeedPosition.get(),
            kRetractMaxVelocityRadiansPerSecond.get(),
            0.0,
            0.0);
    }

    public static IntakeSetpoint churn() {
        return new IntakeSetpoint(
            kFeedPosition.get(),
            kRetractMaxVelocityRadiansPerSecond.get(),
            kIdleVoltage.get(),
            kIdleVelocityRadiansPerSecond.get());
    }

    public static double getSlowMaxVelocityRadiansPerSecond() {
        return kSlowMaxVelocityRadiansPerSecond.get();
    }

    public static double getFastMaxVelocityRadiansPerSecond() {
        return kFastMaxVelocityRadiansPerSecond.get();
    }
}