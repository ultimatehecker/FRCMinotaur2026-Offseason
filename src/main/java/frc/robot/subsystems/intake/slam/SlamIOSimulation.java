package frc.robot.subsystems.intake.slam;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

import frc.robot.Constants;
import frc.robot.constants.IntakeConstants;

public class SlamIOSimulation implements SlamIO {
    private final DCMotor gearbox;
    private final SingleJointedArmSim simulation;
    private double appliedVoltage = 0.0;

    private PIDController controller = new PIDController(0.0, 0.0, 0.0);
    private ArmFeedforward feedforward = new ArmFeedforward(0.0, 0.0, 0.0, 0.0);

    private double currentOutput = 0.0;
    private boolean currentControl = false;

    private double previousKP = 0.0;
    private double previousKD = 0.0;
    private double previousKS = 0.0;
    private double previousKV = 0.0;
    private double previousKG = 0.0;
    private double previousKA = 0.0;

    public SlamIOSimulation() {
        gearbox = IntakeConstants.Slam.kSimulatedGearbox;
        simulation = new SingleJointedArmSim(
            gearbox,
            IntakeConstants.Slam.kMotorReduction,
            IntakeConstants.kMOI,
            IntakeConstants.kLength,
            IntakeConstants.kMinimumPosition,
            IntakeConstants.kMaximumPosition,
            true, 
            IntakeConstants.kMinimumPosition
        );
    }

    @Override
    public void updateInputs(SlamIOInputs inputs) {
        inputs.motorConnected = true;
        inputs.positionRadians = simulation.getAngleRads();
        inputs.velocityRadiansPerSecond = simulation.getVelocityRadPerSec();
        inputs.appliedVoltage = appliedVoltage;
        inputs.torqueCurrentAmperes = gearbox.getCurrent(simulation.getVelocityRadPerSec(), appliedVoltage);
        inputs.supplyCurrentAmperes = simulation.getCurrentDrawAmps();
        inputs.temperatureCelsius = 0.0; 
        inputs.temperatureFault = false;

        if (currentControl) {
            appliedVoltage = MathUtil.clamp(gearbox.getVoltage(currentOutput, simulation.getVelocityRadPerSec()), -12.0, 12.0);
        }

        simulation.setInputVoltage(appliedVoltage);
        simulation.update(Constants.kSimLoopPeriodSeconds);
    }

    @Override
    public void applyOutputs(SlamIOOutputs outputs) {
        if (outputs.kP != previousKP || outputs.kD != previousKD || outputs.kS != previousKS || outputs.kV != previousKV || outputs.kG != previousKG || outputs.kA != previousKA) {
            controller = new PIDController(outputs.kP, 0.0, outputs.kD);
            feedforward = new ArmFeedforward(outputs.kS, outputs.kG, outputs.kV, outputs.kA);

            previousKP = outputs.kP;
            previousKD = outputs.kD;
            previousKS = outputs.kS;
            previousKV = outputs.kV;
            previousKG = outputs.kG;
            previousKA = outputs.kA;
        }

        switch (outputs.mode) {
            case BRAKE:
                currentControl = false;
                appliedVoltage = 0.0;
                break;
            case COAST:
                currentOutput = 0.0;
                currentControl = true;
                break;
            case VOLTAGE_CONTROL:
                currentOutput = outputs.appliedVoltage;
                currentControl = false;
                break;
            case CLOSED_LOOP_SENSORED, CLOSED_LOOP_UNSENSORED:
                currentControl = false;
                appliedVoltage = controller.calculate(simulation.getAngleRads(), outputs.position) + feedforward.calculate(simulation.getAngleRads(), simulation.getVelocityRadPerSec());
                break;
        }
    }
}