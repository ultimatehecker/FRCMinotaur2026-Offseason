// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.minolib.localization.WeightedPoseEstimate;
import frc.robot.simulation.SimulatedRobotState;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainIOHardware;
import frc.robot.subsystems.drivetrain.DrivetrainIOSimulation;
import frc.robot.subsystems.drivetrain.TunerConstants;

public class RobotContainer {
  private final Consumer<WeightedPoseEstimate> visionEstimateConsumer = new Consumer<WeightedPoseEstimate>() {
    @Override
    public void accept(WeightedPoseEstimate estimate) {
        drivetrain.addVisionMeasurement(estimate);
    }
  };

  private RobotState robotState;
  private SimulatedRobotState simulatedRobotState;

  private Drivetrain drivetrain;

  private RobotState buildRobotState() {
    return new RobotState(visionEstimateConsumer);
  }

  private SimulatedRobotState buildSimulatedRobotState() {
    if (Robot.isSimulation()) {
      return new SimulatedRobotState(this);
    } else {
      return null;
    }
  }

  @SuppressWarnings("unchecked")
  private Drivetrain buildDrivetrain() {
    if (Robot.isSimulation()) {
      return new Drivetrain(
        robotState,
        new DrivetrainIOSimulation(robotState, simulatedRobotState, TunerConstants.kDrivetrainConstants, TunerConstants.kFrontLeft, TunerConstants.kFrontRight, TunerConstants.kBackLeft, TunerConstants.kBackRight)
      );
    } else {
      return new Drivetrain(
        robotState, 
        new DrivetrainIOHardware(robotState, TunerConstants.kDrivetrainConstants, TunerConstants.kFrontLeft, TunerConstants.kFrontRight, TunerConstants.kBackLeft, TunerConstants.kBackRight)
      );
    }
  }

  public RobotState getRobotState() {
    return robotState;
  }

  public SimulatedRobotState getSimulatedRobotState() {
    return simulatedRobotState;
  }

  public Drivetrain getDrivetrain() {
    return drivetrain;
  }

  public RobotContainer() {
    robotState = buildRobotState();
    simulatedRobotState = buildSimulatedRobotState();

    drivetrain = buildDrivetrain();

    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
