package frc.robot.subsystems.intake.slam;

import org.littletonrobotics.junction.AutoLog;

public interface SlamIO {
    @AutoLog
    public static class SlamIOInputs {
        public boolean connected;
        public double positionRadians;
        public double velocityRadiansPerSecond;
        public double appliedVoltage;
        public double supplyCurrentAmperes;
        public double torqueCurrentAmperes;
        public double temperatureCelsius;
    }

    public enum SlamIOOutputMode {
        BRAKE,
        COAST,
        RUN_OPEN_LOOP,
        RUN_CLOSED_LOOP
    }

    public static class SlamIOOutputs {
        public SlamIOOutputMode mode = SlamIOOutputMode.BRAKE;
        public double appliedVolts = 0.0;

        public double position = 0.0;
        public double kP = 0.0;
        public double kD = 0.0;
    }

    public default void updateInputs(SlamIOInputs inputs) {}

    public default void applyOutputs(SlamIOOutputs outputs) {}
}