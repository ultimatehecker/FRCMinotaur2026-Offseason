package frc.robot.subsystems.leds;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.CANdleFeaturesConfigs;
import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.Enable5VRailValue;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

import static frc.minolib.utilities.PhoenixUtility.tryUntilOk;

import frc.minolib.utilities.PhoenixUtility;
import frc.robot.constants.LedConstants;

public class LedIOHardware implements LedIO {
    private CANdle candle;
    private CANdleConfiguration configuration;

    private StatusSignal<Voltage> appliedVoltage;
    private StatusSignal<Current> outputCurrent;
    private StatusSignal<Temperature> temperature;
    private StatusSignal<Boolean> temperatureFault;

    public LedIOHardware() {
        candle = new CANdle(LedConstants.kCandle.getDeviceID(), LedConstants.kCandle.getCANBus());
        configuration = new CANdleConfiguration()
            .withLED(
                new LEDConfigs()
                    .withLossOfSignalBehavior(LossOfSignalBehaviorValue.KeepRunning)
                    .withStripType(StripTypeValue.GRBW)
                    .withBrightnessScalar(1.0)
            ).withCANdleFeatures(
                new CANdleFeaturesConfigs()
                    .withEnable5VRail(Enable5VRailValue.Enabled)
                    .withStatusLedWhenActive(StatusLedWhenActiveValue.Enabled)
            );
        
        tryUntilOk(5, () -> candle.getConfigurator().apply(configuration));

        appliedVoltage = candle.getSupplyVoltage();
        outputCurrent = candle.getOutputCurrent();
        temperature = candle.getDeviceTemp();
        temperatureFault = candle.getFault_Thermal();

        tryUntilOk(5, () -> BaseStatusSignal.setUpdateFrequencyForAll(50.0, appliedVoltage, outputCurrent, temperature, temperatureFault));
        tryUntilOk(5, () -> candle.optimizeBusUtilization(0.0, 0.25));

        PhoenixUtility.registerSignals(true, appliedVoltage, outputCurrent, temperature, temperatureFault);
    }

    @Override
    public void updateInputs(LedIOInputs inputs) {
        inputs.connected = BaseStatusSignal.isAllGood(appliedVoltage, outputCurrent, temperature);
        inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
        inputs.supplyCurrentAmperes = outputCurrent.getValueAsDouble();
        inputs.temperatureCelsius = temperature.getValueAsDouble();
        inputs.temperatureFault = temperatureFault.getValue().booleanValue();
    }

    @Override
    public void setAnimation(ControlRequest request) {
        candle.setControl(request);
    }

    @Override
    public void clearAnimation() {
        candle.clearAllAnimations();
    }
}