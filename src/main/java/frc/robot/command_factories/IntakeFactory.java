package frc.robot.command_factories;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.minolib.math.EqualsUtility;
import frc.robot.RobotContainer;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.intake.IntakeSetpoint;

public class IntakeFactory {
    public static Command deployIntakeBlocking(RobotContainer robotContainer) {
        return Commands.startEnd(
            () -> robotContainer.getIntake().setSetpoint(IntakeSetpoint.intake()), 
            () -> {}
        ).until(() -> EqualsUtility.epsilonEquals(
            robotContainer.getIntake().getSlamCurrentPositionDegrees(), 
            robotContainer.getIntake().getSetpoint().slamAngleDegrees(),
            IntakeConstants.kPivotTolerance.in(Degrees)
        ));
    }

    public static Command exhaustIntakeBlocking(RobotContainer robotContainer) {
        return Commands.startEnd(
            () -> robotContainer.getIntake().setSetpoint(IntakeSetpoint.exhaustDeployed()), 
            () -> {}
        ).until(() -> EqualsUtility.epsilonEquals(
            robotContainer.getIntake().getSlamCurrentPositionDegrees(), 
            robotContainer.getIntake().getSetpoint().slamAngleDegrees(),
            IntakeConstants.kPivotTolerance.in(Degrees)
        ));
    }

    public static Command exhaustHalfIntakeBlocking(RobotContainer robotContainer) {
        return Commands.startEnd(
            () -> robotContainer.getIntake().setSetpoint(IntakeSetpoint.exhaustHalfDeployed()), 
            () -> {}
        ).until(() -> EqualsUtility.epsilonEquals(
            robotContainer.getIntake().getSlamCurrentPositionDegrees(), 
            robotContainer.getIntake().getSetpoint().slamAngleDegrees(),
            IntakeConstants.kPivotTolerance.in(Degrees)
        ));
    }

    public static Command wallIntakeBlocking(RobotContainer robotContainer) {
        return Commands.startEnd(
            () -> robotContainer.getIntake().setSetpoint(IntakeSetpoint.wall()), 
            () -> {}
        ).until(() -> EqualsUtility.epsilonEquals(
            robotContainer.getIntake().getSlamCurrentPositionDegrees(), 
            robotContainer.getIntake().getSetpoint().slamAngleDegrees(),
            IntakeConstants.kPivotTolerance.in(Degrees)
        ));
    }

    public static Command churnIntakeBlocking(RobotContainer robotContainer) {
        return Commands.startEnd(
            () -> robotContainer.getIntake().setSetpoint(IntakeSetpoint.churn()), 
            () -> {}
        ).until(() -> EqualsUtility.epsilonEquals(
            robotContainer.getIntake().getSlamCurrentPositionDegrees(), 
            robotContainer.getIntake().getSetpoint().slamAngleDegrees(),
            IntakeConstants.kPivotTolerance.in(Degrees)
        ));
    }

    public static Command slowRetractIntakeBlocking(RobotContainer robotContainer) {
        return Commands.startEnd(
            () -> robotContainer.getIntake().setSetpoint(IntakeSetpoint.slowRetract()), 
            () -> {}
        ).until(() -> EqualsUtility.epsilonEquals(
            robotContainer.getIntake().getSlamCurrentPositionDegrees(), 
            robotContainer.getIntake().getSetpoint().slamAngleDegrees(),
            IntakeConstants.kPivotTolerance.in(Degrees)
        ));
    }

    public static Command retractIntakeBlocking(RobotContainer robotContainer) {
        return Commands.startEnd(
            () -> robotContainer.getIntake().setSetpoint(IntakeSetpoint.fastRetract()), 
            () -> {}
        ).until(() -> EqualsUtility.epsilonEquals(
            robotContainer.getIntake().getSlamCurrentPositionDegrees(), 
            robotContainer.getIntake().getSetpoint().slamAngleDegrees(),
            IntakeConstants.kPivotTolerance.in(Degrees)
        ));
    }
}