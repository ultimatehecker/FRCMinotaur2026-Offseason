package frc.robot.subsystems.rollers;

import org.littletonrobotics.junction.AutoLog;

public interface RollerSystemIO {
    @AutoLog
    public static class RollerSystemIOInputs {
        public boolean connected;
        public double positionRadians;
        public double velocityRadiansPerSecond;
        public double appliedVoltage;
        public double supplyCurrentAmperes;
        public double torqueCurrentAmperes;
        public double temperatureCelsius;

        public boolean hasFollower;
        public boolean followerConnected;
        public double followerSupplyCurrentAmperes;
        public double followerTemperatureCelsius;
    }

    public enum RollerSystemIOMode {
        BRAKE,
        COAST,
        VOLTAGE_CONTROL,
        CLOSED_LOOP
    }

    public static class RollerSystemIOOutputs {
        public RollerSystemIOMode mode = RollerSystemIOMode.BRAKE;
        
        // Voltage control
        public double appliedVoltage = 0.0;

        // Closed loop control
        public double velocity = 0.0;
        public double kP = 0.0;
        public double kD = 0.0;
        public double feedforward = 0.0;

        public boolean brakeModeEnabled = true;
    }

    public default void updateInputs(RollerSystemIOInputs inputs) {}

    public default void applyOutputs(RollerSystemIOOutputs outputs) {}
}