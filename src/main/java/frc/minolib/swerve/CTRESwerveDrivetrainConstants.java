package frc.minolib.swerve;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import frc.robot.Constants;

public class CTRESwerveDrivetrainConstants {
    SwerveDrivetrainConstants driveTrainConstants;
    SwerveModuleConstants<?, ?, ?>[] moduleConstants;

    public CTRESwerveDrivetrainConstants(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
        this.driveTrainConstants = driveTrainConstants;

        if (Constants.kUseMapleSim) {
            this.moduleConstants = MapleSimulatedSwerveDrivetrain.regulateModuleConstantsForSimulation(modules);
        } else {
            this.moduleConstants = modules;
        }
    }

    public SwerveDrivetrainConstants getDriveTrainConstants() {
        return driveTrainConstants;
    }

    public SwerveModuleConstants<?, ?, ?>[] getModuleConstants() {
        return moduleConstants;
    }
}