package frc.robot.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

import frc.minolib.advantagekit.LoggedTunableNumber;
import frc.robot.RobotState;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class DriveToPose extends Command {
    private RobotState robotState;
    private Drivetrain drivetrain;

    private Supplier<Pose2d> targetSupplier;

    private ProfiledPIDController translationController;
    private ProfiledPIDController rotationController;

    private static final LoggedTunableNumber drivekP = new LoggedTunableNumber("Drivetrain/DriveToPose/Drive/kP", DrivetrainConstants.kDriveHolonomickP);
    private static final LoggedTunableNumber drivekI = new LoggedTunableNumber("Drivetrain/DriveToPose/Drive/kI", DrivetrainConstants.kDriveHolonomickI);
    private static final LoggedTunableNumber drivekD = new LoggedTunableNumber("Drivetrain/DriveToPose/Drive/kD", DrivetrainConstants.kDriveHolonomickD);
    private static final LoggedTunableNumber driveMaxVelocity = new LoggedTunableNumber("Drivetrain/DriveToPose/Drive/Max Velocity", DrivetrainConstants.kDriveHolonomicMaxVelocity);
    private static final LoggedTunableNumber driveMaxAcceleration = new LoggedTunableNumber("Drivetrain/DriveToPose/Drive/Max Acceleration", DrivetrainConstants.kDriveHolonomicMaxAcceleration);

    private static final LoggedTunableNumber rotkP = new LoggedTunableNumber("Drivetrain/DriveToPose/Rotation/kP", DrivetrainConstants.kRotationalHolonomickP);
    private static final LoggedTunableNumber rotkI = new LoggedTunableNumber("Drivetrain/DriveToPose/Rotation/kI", DrivetrainConstants.kRotationalHolonomickI);
    private static final LoggedTunableNumber rotkD = new LoggedTunableNumber("Drivetrain/DriveToPose/Rotation/kD", DrivetrainConstants.kRotationalHolonomickD);
    private static final LoggedTunableNumber rotMaxVelocity = new LoggedTunableNumber("Drivetrain/DriveToPose/Rotation/Max Velocity", DrivetrainConstants.kRotationalHolonomicMaxVelocity);
    private static final LoggedTunableNumber rotMaxAcceleration = new LoggedTunableNumber("Drivetrain/DriveToPose/Rotation/Max Acceleration", DrivetrainConstants.kRotationalHolonomicMaxAcceleration);

    public DriveToPose(RobotState robotState, Drivetrain drivetrain, Supplier<Pose2d> targetSupplier) {
        this.drivetrain = drivetrain;
        this.robotState = robotState;
        this.targetSupplier = targetSupplier;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        translationController = new ProfiledPIDController(
            drivekP.get(),
            drivekI.get(),
            drivekD.get(),
            new TrapezoidProfile.Constraints(
                driveMaxVelocity.get(),
                driveMaxAcceleration.get()
            )
        );

        rotationController = new ProfiledPIDController(
            rotkP.get(),
            rotkI.get(),
            rotkD.get(),
            new TrapezoidProfile.Constraints(
                rotMaxVelocity.get(),
                rotMaxAcceleration.get()
            )
        );

        rotationController.enableContinuousInput(-Math.PI, Math.PI);

        Pose2d targetPose = targetSupplier.get();
        Pose2d currentPose = robotState.getLatestFieldToRobot().getValue();

        ChassisSpeeds currentSpeeds = robotState.getLatestMeasuredFieldRelativeChassisSpeeds();

        Translation2d translationError = targetPose.minus(currentPose).getTranslation();
        Rotation2d directionToTarget = translationError.getAngle();

        double velocityTowardTarget = currentSpeeds.vxMetersPerSecond * directionToTarget.getCos() + currentSpeeds.vyMetersPerSecond * directionToTarget.getSin();

        translationController.reset(translationError.getNorm(), -velocityTowardTarget);

        rotationController.reset(currentPose.getRotation().getRadians(), currentSpeeds.omegaRadiansPerSecond);
    }

    @Override
    public void execute() {
        if (drivekP.hasChanged(hashCode()) || drivekI.hasChanged(hashCode()) || drivekD.hasChanged(hashCode())) {
            translationController.setPID(drivekP.get(), drivekI.get(), drivekD.get());
        }

        if (rotkP.hasChanged(hashCode()) || rotkI.hasChanged(hashCode()) || rotkD.hasChanged(hashCode())) {
            rotationController.setPID(rotkP.get(), rotkI.get(), rotkD.get());
        }

        if (driveMaxVelocity.hasChanged(hashCode()) || driveMaxAcceleration.hasChanged(hashCode())) {
            translationController.setConstraints(new TrapezoidProfile.Constraints(driveMaxVelocity.get(), driveMaxAcceleration.get()));
        }

        if (rotMaxVelocity.hasChanged(hashCode()) || rotMaxAcceleration.hasChanged(hashCode())) {
            rotationController.setConstraints(new TrapezoidProfile.Constraints(rotMaxVelocity.get(), rotMaxAcceleration.get()));
        }

        Pose2d targetPose = targetSupplier.get();
        Pose2d currentPose = robotState.getLatestFieldToRobot().getValue();

        Translation2d translationError = targetPose.minus(currentPose).getTranslation();

        Rotation2d directionToTarget = translationError.getAngle();

        double translationOutput = translationController.calculate(translationError.getNorm(), 0.0);

        double rotationOutput = rotationController.calculate(
            currentPose.getRotation().getRadians(),
            targetPose.getRotation().getRadians()
        );

        Translation2d translationVelocity = new Translation2d(-translationOutput, directionToTarget);

        drivetrain.applyRequest(drivetrain.robotVelocityRequest.withSpeeds(
            new ChassisSpeeds(
                translationVelocity.getX(),
                translationVelocity.getY(),
                rotationOutput
            )
        ));

        Logger.recordOutput("DriveToPose/Target Pose", targetPose);
        Logger.recordOutput("DriveToPose/Translation Output", translationOutput);
        Logger.recordOutput("DriveToPose/Rotation Output", rotationOutput);
        Logger.recordOutput("DriveToPose/Translation Error", translationError.getNorm());
        Logger.recordOutput("DriveToPose/Rotation Error", targetPose.getRotation().minus(currentPose.getRotation()).getRadians());
        Logger.recordOutput("DriveToPose/Translation Velocity", translationVelocity);
        Logger.recordOutput("DriveToPose/Direction to Target", directionToTarget);
    }

    @Override
    public boolean isFinished() {
        Pose2d currentPose = robotState.getLatestFieldToRobot().getValue();
        return MathUtil.isNear(targetSupplier.get().getX(), currentPose.getX(), Units.inchesToMeters(0.5)) &&  MathUtil.isNear(targetSupplier.get().getY(), currentPose.getY(), Units.inchesToMeters(0.5));
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}