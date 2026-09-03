package frc.robot.subsystems.rollers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class RollerSystemIOSimulation implements RollerSystemIO {
    private final DCMotorSim simulation;
    private final DCMotor gearbox;
    private double appliedVoltage = 0.0;
    private boolean hasFollower;

    public RollerSystemIOSimulation(DCMotor motorModel, double reduction, double moi, boolean hasFollower) {
        gearbox = motorModel;
        simulation = new DCMotorSim(LinearSystemId.createDCMotorSystem(motorModel, moi, reduction), motorModel);
        
        this.hasFollower = hasFollower;
    }

    @Override
    public void updateInputs(RollerSystemIOInputs inputs) {
        simulation.update(Constants.kSimLoopPeriodSeconds);

        inputs.connected = true;
        if (hasFollower) {
            inputs.followerConnected = true;
            inputs.hasFollower = true;
        }

        inputs.positionRadians = simulation.getAngularPositionRad();
        inputs.velocityRadiansPerSecond = simulation.getAngularVelocityRadPerSec();
        inputs.appliedVoltage = appliedVoltage;
        inputs.supplyCurrentAmperes = simulation.getCurrentDrawAmps();
        inputs.torqueCurrentAmperes = gearbox.getCurrent(simulation.getAngularVelocityRadPerSec(), appliedVoltage);
        inputs.temperatureCelsius = 0.0;
    }

    @Override
    public void applyOutputs(RollerSystemIOOutputs outputs) {
        if (DriverStation.isDisabled()) {
            appliedVoltage = 0.0;
        } else {
            appliedVoltage = MathUtil.clamp(outputs.appliedVoltage, -12.0, 12.0);
        }

        simulation.setInputVoltage(appliedVoltage);
    }
}