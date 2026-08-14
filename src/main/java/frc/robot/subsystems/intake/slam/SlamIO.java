package frc.robot.subsystems.intake.slam;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.signals.MagnetHealthValue;

public interface SlamIO {
    @AutoLog
    public static class SlamIOInputs {
        public boolean motorConnected;
        public double positionRadians;
        public double velocityRadiansPerSecond;
        public double appliedVoltage;
        public double supplyCurrentAmperes;
        public double torqueCurrentAmperes;
        public double temperatureCelsius;
        public boolean temperatureFault;

        public boolean absoluteEncoderConnected;
        public double absoluteEncoderPositionRadians;
        public MagnetHealthValue absoluteEncoderMagnetHealth;
    }

    public enum SlamIOOutputMode {
        BRAKE,
        COAST,
        VOLTAGE_CONTROL,
        CLOSED_LOOP_SENSORED,
        CLOSED_LOOP_UNSENSORED
    }

    public static class SlamIOOutputs {
        public SlamIOOutputMode mode = SlamIOOutputMode.BRAKE;
        public double appliedVoltage = 0.0;

        public double position = 0.0;
        public double kP = 0.0;
        public double kD = 0.0;
        public double kS = 0.0;
        public double kV = 0.0;
        public double kG = 0.0;
        public double kA = 0.0;
    }

    public default void updateInputs(SlamIOInputs inputs) {}

    public default void applyOutputs(SlamIOOutputs outputs) {}
}