package frc.robot.subsystems.intake.slam;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

import static frc.minolib.utilities.PhoenixUtility.tryUntilOk;

import frc.minolib.utilities.PhoenixUtility;
import frc.robot.constants.IntakeConstants;

public class SlamIOHardware implements SlamIO {
    private final TalonFX motor;
    private final CANcoder absoluteEncoder;

    private final TalonFXConfiguration motorConfiguration;
    private final CANcoderConfiguration absoluteEncoderConfiguration;

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> appliedVoltage;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Current> torqueCurrent;
    private final StatusSignal<Temperature> temperature;
    private final StatusSignal<Boolean> temperatureFault;

    private final StatusSignal<Angle> absolutePosition;
    private final StatusSignal<MagnetHealthValue> absoluteMagnetHealth;

    private final VoltageOut voltageRequest = new VoltageOut(0.0)
        .withEnableFOC(true)
        .withUpdateFreqHz(0);

    private final PositionVoltage positionRequest = new PositionVoltage(0.0)
        .withEnableFOC(true)
        .withUpdateFreqHz(0);

    private final CoastOut coastRequest = new CoastOut();
    private final StaticBrake brakeRequest = new StaticBrake();

    private double previousKP = 0.0;
    private double previousKD = 0.0;
    private double previousKS = 0.0;
    private double previousKV = 0.0;
    private double previousKG = 0.0;
    private double previousKA = 0.0;

    private boolean usingAbsoluteEncoder = false;
    private boolean previousUsingAbsoluteEncoder = false;

    public SlamIOHardware() {
        motor = new TalonFX(IntakeConstants.Slam.kKrakenX60.getDeviceID(), IntakeConstants.Slam.kKrakenX60.getCANBus());
        absoluteEncoder = new CANcoder(IntakeConstants.Slam.kCANCoder.getDeviceID(), IntakeConstants.Slam.kCANCoder.getCANBus());

        absoluteEncoderConfiguration = new CANcoderConfiguration()
            .withMagnetSensor(
                new MagnetSensorConfigs()
                    .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
                    .withAbsoluteSensorDiscontinuityPoint(0.5)
                    .withMagnetOffset(Units.radiansToRotations(IntakeConstants.Slam.kAbsoluteEncoderOffsetRadians))
            );

        tryUntilOk(5, () -> absoluteEncoder.getConfigurator().apply(absoluteEncoderConfiguration, 0.25));

        motorConfiguration = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(IntakeConstants.Slam.kMotorInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake)
            ).withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(true)
                    .withStatorCurrentLimit(IntakeConstants.Slam.kMotorStatorCurrentLimit)
                    .withSupplyCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(IntakeConstants.Slam.kMotorSupplyCurrentLimit)
            ).withFeedback(
                new FeedbackConfigs()
                    .withSensorToMechanismRatio(IntakeConstants.Slam.kMotorReduction)
                    .withFusedCANcoder(usingAbsoluteEncoder ? absoluteEncoder : null)
            ).withAudio(
                new AudioConfigs()
                    .withBeepOnBoot(false)   
                    .withBeepOnConfig(false)
            );

        tryUntilOk(5, () -> motor.getConfigurator().apply(motorConfiguration, 0.25));

        position = motor.getPosition();
        velocity = motor.getVelocity();
        appliedVoltage = motor.getMotorVoltage();
        supplyCurrent = motor.getSupplyCurrent();
        torqueCurrent = motor.getTorqueCurrent();
        temperature = motor.getDeviceTemp();
        temperatureFault = motor.getFault_DeviceTemp();

        absolutePosition = absoluteEncoder.getAbsolutePosition();
        absoluteMagnetHealth = absoluteEncoder.getMagnetHealth();

        tryUntilOk(5, () -> BaseStatusSignal.setUpdateFrequencyForAll(50.0, position, velocity, appliedVoltage, supplyCurrent, torqueCurrent, temperature, temperatureFault, absolutePosition, absoluteMagnetHealth));
        tryUntilOk(5, () -> motor.optimizeBusUtilization(0.0, 0.25));

        PhoenixUtility.registerSignals(false, position, velocity, appliedVoltage, supplyCurrent, torqueCurrent, temperature, temperatureFault, absolutePosition, absoluteMagnetHealth);
    }

    @Override
    public void updateInputs(SlamIOInputs inputs) {
        inputs.motorConnected = BaseStatusSignal.isAllGood(position, velocity, appliedVoltage, supplyCurrent, torqueCurrent, temperature);
        inputs.positionRadians = Units.rotationsToRadians(position.getValueAsDouble());
        inputs.velocityRadiansPerSecond = Units.rotationsToRadians(velocity.getValueAsDouble());
        inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
        inputs.supplyCurrentAmperes = supplyCurrent.getValueAsDouble();
        inputs.torqueCurrentAmperes = torqueCurrent.getValueAsDouble();
        inputs.temperatureCelsius = temperature.getValueAsDouble();
        inputs.temperatureFault = temperatureFault.getValue().booleanValue();

        if (usingAbsoluteEncoder) {
            inputs.absoluteEncoderConnected = BaseStatusSignal.isAllGood(absolutePosition, absoluteMagnetHealth);
            inputs.absoluteEncoderPositionRadians = Units.rotationsToRadians(absolutePosition.getValueAsDouble());
            inputs.absoluteEncoderMagnetHealth = absoluteMagnetHealth.getValue();
        }
    }

    @Override
    public void applyOutputs(SlamIOOutputs outputs) {
        if (outputs.kP != previousKP || outputs.kD != previousKD || outputs.kS != previousKS || outputs.kV != previousKV || outputs.kG != previousKG || outputs.kA != previousKA) {
            motorConfiguration.Slot0 = new Slot0Configs()
                .withKP(outputs.kP)
                .withKD(outputs.kD)
                .withKS(outputs.kS)
                .withKV(outputs.kV)
                .withKG(outputs.kG)
                .withKA(outputs.kA)
                .withGravityType(GravityTypeValue.Arm_Cosine);

            PhoenixUtility.tryUntilOk(5, () -> motor.getConfigurator().apply(motorConfiguration.Slot0, 0.0));

            previousKP = outputs.kP;
            previousKD = outputs.kD;
            previousKS = outputs.kS;
            previousKV = outputs.kV;
            previousKG = outputs.kG;
            previousKA = outputs.kA;
        }

        if (usingAbsoluteEncoder != previousUsingAbsoluteEncoder) {
            motorConfiguration.Feedback = new FeedbackConfigs()
                .withSensorToMechanismRatio(IntakeConstants.Slam.kMotorReduction)
                .withFusedCANcoder(usingAbsoluteEncoder ? absoluteEncoder : null);

            PhoenixUtility.tryUntilOk(5, () -> motor.getConfigurator().apply(motorConfiguration.Feedback, 0.0));

            previousUsingAbsoluteEncoder = usingAbsoluteEncoder;
        }

        switch (outputs.mode) {
            case BRAKE -> motor.setControl(brakeRequest);
            case COAST -> motor.setControl(coastRequest);
            case VOLTAGE_CONTROL -> motor.setControl(voltageRequest.withOutput(outputs.appliedVoltage));
            case CLOSED_LOOP_SENSORED -> motor.setControl(positionRequest.withPosition(Units.radiansToRotations(outputs.position)));
            case CLOSED_LOOP_UNSENSORED -> motor.setControl(positionRequest.withPosition(Units.radiansToRotations(outputs.position)));
        }
    }
}