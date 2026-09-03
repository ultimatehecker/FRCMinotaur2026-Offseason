package frc.robot.subsystems.rollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

import static frc.minolib.utilities.PhoenixUtility.tryUntilOk;

import frc.minolib.hardware.MinoCANDevice;
import frc.minolib.utilities.PhoenixUtility;

public class RollerSystemIOTalonFX implements RollerSystemIO {
    private final TalonFX motor;
    private final TalonFX followerMotor;
    private final TalonFXConfiguration configuration;

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> appliedVoltage;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Current> torqueCurrent;
    private final StatusSignal<Temperature> temperature;
    private final StatusSignal<Boolean> temperatureFault;

    private final StatusSignal<Current> followerSupplyCurrent;
    private final StatusSignal<Temperature> followerTemperature;

    private final VoltageOut voltageRequest = new VoltageOut(0.0)
        .withEnableFOC(true)
        .withUpdateFreqHz(0);

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0)
        .withEnableFOC(true)
        .withUpdateFreqHz(0);

    private final CoastOut coastRequest = new CoastOut();
    private final StaticBrake brakeRequest = new StaticBrake();

    private double previousKP = 0.0;
    private double previousKD = 0.0;

    public RollerSystemIOTalonFX(String name, MinoCANDevice device, MinoCANDevice followerDevice, boolean inverted, boolean followerAligned, double reduction, double statorCurrentLimit, double supplyCurrentLimit) {
        motor = new TalonFX(device.getDeviceID(), device.getCANBus());
        followerMotor = new TalonFX(followerDevice.getDeviceID(), followerDevice.getCANBus());

        configuration = new TalonFXConfiguration();

        configuration
            .withMotorOutput(new MotorOutputConfigs()
                .withInverted(inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake)
            ).withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(statorCurrentLimit)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(supplyCurrentLimit)
            ).withFeedback(
                new FeedbackConfigs()
                    .withVelocityFilterTimeConstant(0.1)
                    .withSensorToMechanismRatio(reduction)
            );

        tryUntilOk(5, () -> motor.getConfigurator().apply(configuration, 0.25));

        if (followerMotor != null) {
            tryUntilOk(5, () -> followerMotor.getConfigurator().apply(configuration, 0.25));
        }

        position = motor.getPosition();
        velocity = motor.getVelocity();
        appliedVoltage = motor.getMotorVoltage();
        supplyCurrent = motor.getSupplyCurrent();
        torqueCurrent = motor.getTorqueCurrent();
        temperature = motor.getDeviceTemp();
        temperatureFault = motor.getFault_DeviceTemp();

        if (followerMotor != null) {
            followerSupplyCurrent = followerMotor.getSupplyCurrent();
            followerTemperature = followerMotor.getDeviceTemp();
        } else {
            followerSupplyCurrent = null;
            followerTemperature = null;
        }

        tryUntilOk(5, () -> BaseStatusSignal.setUpdateFrequencyForAll(50.0, position, velocity, appliedVoltage, supplyCurrent, torqueCurrent, temperature, temperatureFault));
        tryUntilOk(5, () -> motor.optimizeBusUtilization(0.0, 0.25));

        PhoenixUtility.registerSignals(false, position, velocity, appliedVoltage, supplyCurrent, torqueCurrent, temperature, temperatureFault);

        if (followerMotor != null) {
            tryUntilOk(5, () -> BaseStatusSignal.setUpdateFrequencyForAll(50.0, followerSupplyCurrent, followerTemperature));
            tryUntilOk(5, () -> followerMotor.optimizeBusUtilization(0.0, 0.25));

            PhoenixUtility.registerSignals(false, followerSupplyCurrent, followerTemperature);

            followerMotor.setControl(new Follower(motor.getDeviceID(), followerAligned ? MotorAlignmentValue.Aligned : MotorAlignmentValue.Opposed)
                .withUpdateFreqHz(20.0));
        }
    }

    @Override
    public void updateInputs(RollerSystemIOInputs inputs) {
        inputs.connected = BaseStatusSignal.isAllGood(position, velocity, appliedVoltage, supplyCurrent, torqueCurrent, temperature);
        inputs.positionRadians = Units.rotationsToRadians(position.getValueAsDouble());
        inputs.velocityRadiansPerSecond = Units.rotationsToRadians(velocity.getValueAsDouble());
        inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
        inputs.supplyCurrentAmperes = supplyCurrent.getValueAsDouble();
        inputs.torqueCurrentAmperes = torqueCurrent.getValueAsDouble();
        inputs.temperatureCelsius = temperature.getValueAsDouble();

        inputs.hasFollower = followerMotor != null;

        if (followerMotor != null) {
            inputs.followerConnected = BaseStatusSignal.isAllGood(followerSupplyCurrent, followerTemperature);
            inputs.followerSupplyCurrentAmperes = followerSupplyCurrent.getValueAsDouble();
            inputs.followerTemperatureCelsius = followerTemperature.getValueAsDouble();
        }
    }

    @Override
    public void applyOutputs(RollerSystemIOOutputs outputs) {
        if (outputs.kP != previousKP || outputs.kD != previousKD) {

            configuration.Slot0 = new Slot0Configs().withKP(outputs.kP).withKD(outputs.kD);

            PhoenixUtility.tryUntilOk(5, () -> motor.getConfigurator().apply(configuration.Slot0, 0.0));

            previousKP = outputs.kP;
            previousKD = outputs.kD;
        }

        switch (outputs.mode) {
            case BRAKE -> motor.setControl(brakeRequest);
            case COAST -> motor.setControl(coastRequest);
            case VOLTAGE_CONTROL -> motor.setControl(voltageRequest.withOutput(outputs.appliedVoltage));
            case CLOSED_LOOP -> {
                motor.setControl(velocityRequest
                    .withVelocity(outputs.velocity)
                    .withFeedForward(outputs.feedforward));
            }
        }
    }
}
