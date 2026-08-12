package frc.robot.subsystems.drivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.minolib.advantagekit.LoggedTracer;
import frc.minolib.controller.ControllerConstants;
import frc.minolib.localization.WeightedPoseEstimate;
import frc.minolib.swerve.MapleSimulatedSwerveDrivetrain;
import frc.robot.RobotState;
import frc.robot.constants.DrivetrainConstants;

public class Drivetrain extends SubsystemBase {
    private final RobotState robotState;

    private final DrivetrainIO io;
    private final DrivetrainIOInputsAutoLogged inputs = new DrivetrainIOInputsAutoLogged();

    private final String[] moduleNames = {"Drivetrain/FL", "Drivetrain/FR", "Drivetrain/BL", "Drivetrain/BR"};

    private final ModuleIOInputsAutoLogged[] moduleInputs = new ModuleIOInputsAutoLogged[] { 
        new ModuleIOInputsAutoLogged(), 
        new ModuleIOInputsAutoLogged(), 
        new ModuleIOInputsAutoLogged(), 
        new ModuleIOInputsAutoLogged()
     };

    private final SwerveRequest.FieldCentricFacingAngle continuousTracking = new SwerveRequest.FieldCentricFacingAngle()
        .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
        .withHeadingPID(3.0, 0.0, 0.15) 
        .withDeadband(0.02);

    private final SwerveRequest.FieldCentric teleopRequestFC = new SwerveRequest.FieldCentric()
        .withDesaturateWheelSpeeds(true)
        .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo);

    private final SwerveRequest.RobotCentric teleopRequestRC = new SwerveRequest.RobotCentric()
        .withDesaturateWheelSpeeds(true)
        .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo);

    public final SwerveRequest.ApplyRobotSpeeds robotVelocityRequest = new SwerveRequest.ApplyRobotSpeeds()
        .withDesaturateWheelSpeeds(true)
        .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
        .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo);

    private final SwerveRequest.SwerveDriveBrake idleRequest = new SwerveRequest.SwerveDriveBrake()
        .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo);

    private final ChassisSpeeds zeroChassisSpeeds = new ChassisSpeeds(0, 0, 0);

    private boolean isFieldCentric = true;
    private boolean fieldCentricPreviousState = false;

    public Drivetrain(RobotState robotState, DrivetrainIO io) {
        this.robotState = robotState;
        this.io = io;

        continuousTracking.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs, moduleInputs);
        io.logModules(inputs);

        for (int i = 0; i < moduleInputs.length; i++) {
            Logger.processInputs(moduleNames[i], moduleInputs[i]);
        }

        Logger.processInputs("Drivetrain", inputs);
        LoggedTracer.record("DrivetrainPeriodic");
    }

    public Command drive(DoubleSupplier throttleSupplier, DoubleSupplier strafeSupplier, DoubleSupplier rotationSupplier, BooleanSupplier isFieldCentric) {
        return runEnd(() -> {
            ChassisSpeeds speeds = calculateSpeedsBasedOnJoystickInputs(throttleSupplier, strafeSupplier, rotationSupplier);

            boolean fieldCentricCurrentState = isFieldCentric.getAsBoolean();

            if (fieldCentricCurrentState && !fieldCentricPreviousState) {
                this.isFieldCentric = !this.isFieldCentric;
            }

            fieldCentricPreviousState = fieldCentricCurrentState;

            if (this.isFieldCentric) {
                applyRequest(teleopRequestFC
                    .withVelocityX(speeds.vxMetersPerSecond)
                    .withVelocityY(speeds.vyMetersPerSecond)
                    .withRotationalRate(speeds.omegaRadiansPerSecond)
                );
            } else {
                applyRequest(teleopRequestRC
                    .withVelocityX(speeds.vxMetersPerSecond)
                    .withVelocityY(speeds.vyMetersPerSecond)
                    .withRotationalRate(speeds.omegaRadiansPerSecond)
                );
            }
        }, this::stop).withName("Standard Teleop Drive");
    }

    public Command driveFacingAngle(DoubleSupplier fieldVelocityX, DoubleSupplier fieldVelocityY, Supplier<Rotation2d> targetHeadingSupplier) {
        return runEnd(() -> applyRequest(continuousTracking
            .withVelocityX(fieldVelocityX.getAsDouble())
            .withVelocityY(fieldVelocityY.getAsDouble())
            .withTargetDirection(targetHeadingSupplier.get())
        ), this::stop).withName("Drive Facing Angle");
    }

    public Command xLock() {
        return startEnd(() -> applyRequest(idleRequest), this::stop).withName("X Lock");
    }

    public void applyRequest(SwerveRequest request) {
        io.setSwerveRequest(request);
    }

    public void stop() {
        applyRequest(robotVelocityRequest.withSpeeds(zeroChassisSpeeds));
    }

    public void addVisionMeasurement(WeightedPoseEstimate poseEstimate) {
        io.addVisionMeasurement(poseEstimate);
    }

    public void resetPose(Pose2d pose) {
        io.resetPose(pose);
    }

    public void resetRotationBasedOnAlliance() {
        io.resetRotation();
    }

    public MapleSimulatedSwerveDrivetrain getMapleSimDrive() {
        if (io instanceof DrivetrainIOSimulation simIo) {
            return simIo.getMapleSimDrive();
        }
        return null;
    }

    private ChassisSpeeds calculateSpeedsBasedOnJoystickInputs(DoubleSupplier throttle, DoubleSupplier strafe, DoubleSupplier omega) {
        if (DriverStation.getAlliance().isEmpty()) {
            return new ChassisSpeeds(0, 0, 0);
        }

        double magnitudeX = MathUtil.applyDeadband(throttle.getAsDouble(), ControllerConstants.kControllerDeadband);
        double magnitudeY = MathUtil.applyDeadband(strafe.getAsDouble(), ControllerConstants.kControllerDeadband);
        double magnitudeTheta = MathUtil.applyDeadband(omega.getAsDouble(), ControllerConstants.kControllerDeadband);

        double velocityX = magnitudeX * DrivetrainConstants.kMaximumLinearVelocityMetersPerSecond;
        double velocityY = magnitudeY * DrivetrainConstants.kMaximumLinearVelocityMetersPerSecond;
        double velocityTheta = magnitudeTheta * DrivetrainConstants.kMaximumRotationalVelocityRadiansPerSecond;

        Rotation2d skewCompensationFactor = Rotation2d.fromRadians(robotState.getLatestMeasuredRobotRelativeChassisSpeeds().omegaRadiansPerSecond * -0.03);

        return ChassisSpeeds.fromRobotRelativeSpeeds(
            ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(velocityX, velocityY, velocityTheta), robotState.getLatestFieldToRobot().getValue().getRotation()), 
            robotState.getLatestFieldToRobot().getValue().getRotation().plus(skewCompensationFactor)
        );
    }
}