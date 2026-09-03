package frc.robot.subsystems.leds;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.signals.LarsonBounceValue;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotState;
import frc.robot.constants.LedConstants;

public class Led extends SubsystemBase {
    public enum WantedState {
        DISPLAY_OFF,
        DISPLAY_PHYSICALLY_READY_FOR_MATCH,
        DISPLAY_READY_FOR_MATCH,
        DISPLAY_HARDWARE_FAULT,
        DISPLAY_INTAKING,
        DISPLAY_EXHAUSTING,
        DISPLAY_HOLDING_BALL,
        DISPLAY_PREPARING_TO_SHOOT,
        DISPLAY_SHOOTING,
        DISPLAY_PASSING,
        DISPLAY_CLIMBING,
        DISPLAY_CLIMB_COMPLETE,
        DISPLAY_CONTROLLERS_ACTIVE
    }

    private enum SystemState {
        DISPLAYING_OFF,
        DISPLAYING_PHYSICALLY_READY_FOR_MATCH,
        DISPLAYING_READY_FOR_MATCH,
        DISPLAYING_HARDWARE_FAULT,
        DISPLAYING_INTAKING,
        DISPLAYING_EXHAUSTING,
        DISPLAYING_HOLDING_BALL,
        DISPLAYING_PREPARING_TO_SHOOT,
        DISPLAYING_SHOOTING,
        DISPLAYING_PASSING,
        DISPLAYING_CLIMBING,
        DISPLAYING_CLIMB_COMPLETE,
        DISPLAYING_CONTROLLERS_ACTIVE
    }

    private RobotState robotState;

    private final LedIO io;
    private final LedIOInputsAutoLogged inputs = new LedIOInputsAutoLogged();

    private WantedState wantedState = WantedState.DISPLAY_OFF;
    private SystemState systemState = SystemState.DISPLAYING_OFF;

    public Led(RobotState robotState, LedIO io) {
        this.robotState = robotState;
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Leds", inputs);

        systemState = handleStateTransition();

        Logger.recordOutput("Leds/WantedState", wantedState);
        Logger.recordOutput("Leds/SystemState", systemState);

        applyStates();
    }

    public SystemState handleStateTransition() {
        return switch(wantedState) {
            case DISPLAY_OFF -> SystemState.DISPLAYING_OFF;
            case DISPLAY_PHYSICALLY_READY_FOR_MATCH -> SystemState.DISPLAYING_PHYSICALLY_READY_FOR_MATCH;
            case DISPLAY_READY_FOR_MATCH -> SystemState.DISPLAYING_READY_FOR_MATCH;
            case DISPLAY_HARDWARE_FAULT -> SystemState.DISPLAYING_HARDWARE_FAULT;
            case DISPLAY_INTAKING -> SystemState.DISPLAYING_INTAKING;
            case DISPLAY_EXHAUSTING -> SystemState.DISPLAYING_EXHAUSTING;
            case DISPLAY_HOLDING_BALL -> SystemState.DISPLAYING_HOLDING_BALL;
            case DISPLAY_PREPARING_TO_SHOOT -> SystemState.DISPLAYING_PREPARING_TO_SHOOT;
            case DISPLAY_PASSING -> SystemState.DISPLAYING_PASSING;
            case DISPLAY_SHOOTING -> SystemState.DISPLAYING_SHOOTING;
            case DISPLAY_CLIMBING -> SystemState.DISPLAYING_CLIMBING;
            case DISPLAY_CLIMB_COMPLETE -> SystemState.DISPLAYING_CLIMB_COMPLETE;
            case DISPLAY_CONTROLLERS_ACTIVE -> SystemState.DISPLAYING_CONTROLLERS_ACTIVE;
            default -> SystemState.DISPLAYING_OFF;
        };
    }

    public void applyStates() {
        switch (systemState) {
            case DISPLAYING_OFF:
                io.clearAnimation();
                io.setAnimation(
                    new SolidColor(0, LedConstants.kTotalLedLength)
                );
                break;
            case DISPLAYING_PHYSICALLY_READY_FOR_MATCH:
                io.setAnimation(
                    new LarsonAnimation(0, LedConstants.kTotalLedLength)
                        .withSize(7)
                        .withBounceMode(LarsonBounceValue.Center)
                        .withColor(new RGBWColor(0, 255, 0, 0))
                );
                break;
            case DISPLAYING_READY_FOR_MATCH:
                io.setAnimation(
                    new LarsonAnimation(0, LedConstants.kTotalLedLength)
                        .withSize(7)
                        .withBounceMode(LarsonBounceValue.Center)
                        .withColor(robotState.isRedAlliance() ? new RGBWColor(255, 0, 0, 0) : new RGBWColor(0, 0, 255, 0))
                );
                break;
            case DISPLAYING_HARDWARE_FAULT:
                io.setAnimation(
                    new StrobeAnimation(0, LedConstants.kTotalLedLength)
                        .withColor(new RGBWColor(255, 140, 0, 0))
                );
                break;
            case DISPLAYING_INTAKING:
                io.setAnimation(
                    new SolidColor(0, LedConstants.kTotalLedLength)
                        .withColor(new RGBWColor(255, 255, 0, 0))
                );
                break;
            case DISPLAYING_EXHAUSTING:
                io.setAnimation(
                    new StrobeAnimation(0, LedConstants.kTotalLedLength)
                        .withColor(new RGBWColor(255, 255, 0, 0))
                );
                break;
            case DISPLAYING_HOLDING_BALL:
                io.setAnimation(
                    new SolidColor(0, LedConstants.kTotalLedLength)
                        .withColor(new RGBWColor(255, 0, 0, 0))
                );
                break;
            case DISPLAYING_PREPARING_TO_SHOOT:
                io.setAnimation(
                    new StrobeAnimation(0, LedConstants.kTotalLedLength)
                        .withColor(new RGBWColor(0, 255, 0, 0))
                );
                break;
            case DISPLAYING_SHOOTING:
                io.setAnimation(
                    new SolidColor(0, LedConstants.kTotalLedLength)
                        .withColor(new RGBWColor(0, 255, 0, 0))
                );
                break;
            case DISPLAYING_PASSING:
                io.setAnimation(
                    new SolidColor(0, LedConstants.kTotalLedLength)
                        .withColor(new RGBWColor(0, 0, 255, 0))
                );
                break;
            case DISPLAYING_CLIMBING:
                io.setAnimation(
                    new StrobeAnimation(0, LedConstants.kTotalLedLength)
                        .withColor(new RGBWColor(255, 0, 255, 0))
                );
                break;
            case DISPLAYING_CLIMB_COMPLETE:
                io.setAnimation(
                    new SolidColor(0, LedConstants.kTotalLedLength)
                        .withColor(new RGBWColor(255, 0, 255, 0))
                );
                break;
            case DISPLAYING_CONTROLLERS_ACTIVE:
                io.setAnimation(
                    new LarsonAnimation(0, LedConstants.kTotalLedLength)
                        .withSize(7)
                        .withBounceMode(LarsonBounceValue.Center)
                        .withColor(new RGBWColor(0, 0, 0, 255))
                );
                break;
        }
    }
}
