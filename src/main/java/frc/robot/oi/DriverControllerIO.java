package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface DriverControllerIO {
    public double getThrottle();

    public double getStrafe();

    public double getRotation();

    public double getRotationY();

    public Trigger resetGyro();

    public Trigger toggleDrivingMode();
}