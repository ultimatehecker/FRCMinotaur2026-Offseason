package frc.robot.subsystems.leds;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.controls.ControlRequest;

public interface LedIO {
    @AutoLog
    public class LedIOInputs {
        public boolean connected = false;
        public double appliedVoltage = 0.0;
        public double supplyCurrentAmperes = 0.0;
        public double temperatureCelsius = 0.0;
        public boolean temperatureFault = false;
    }

    public void updateInputs(LedIOInputs inputs);

    public void setAnimation(ControlRequest request);

    public void clearAnimation();
}
