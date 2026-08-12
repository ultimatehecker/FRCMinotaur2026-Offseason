package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.minolib.controller.CommandSimulatedXboxController;
import frc.minolib.controller.ControllerConstants;
import frc.robot.Robot;

public class DriverControllerXbox implements DriverControllerIO {
    private final CommandXboxController controller;

    public DriverControllerXbox() {
        if (Robot.isSimulation()) {
            controller = new CommandSimulatedXboxController(ControllerConstants.kDriverControllerPort); //TODO: Change to simulated controller at a later date
        } else {
            controller = new CommandXboxController(ControllerConstants.kDriverControllerPort);
        }
    }

    @Override
    public double getThrottle() {
        return -(Math.pow(Math.abs(controller.getLeftY()), 1.5)) * Math.signum(controller.getLeftY());
    }

    @Override
    public double getStrafe() {
        return -(Math.pow(Math.abs(controller.getLeftX()), 1.5)) * Math.signum(controller.getLeftX());
    }

    @Override
    public double getRotation() {
        return -(Math.pow(Math.abs(controller.getRightX()), 2.0)) * Math.signum(controller.getRightX());
    }

    @Override
    public double getRotationY() {
        return -(Math.pow(Math.abs(controller.getRightY()), 2.0)) * Math.signum(controller.getRightY());
    }

    @Override
    public Trigger resetGyro() {
        return controller.back().and(controller.start().negate());
    }

    @Override
    public Trigger toggleDrivingMode() {
        return controller.start();
    }
}