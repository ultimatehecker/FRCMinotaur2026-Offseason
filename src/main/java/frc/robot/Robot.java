// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.minolib.advantagekit.LoggedTracer;
import frc.minolib.hardware.MinoCANBus;
import frc.minolib.io.BatteryIOInputsAutoLogged;
import frc.minolib.utilities.BatteryLogger;
import frc.minolib.utilities.PhoenixUtility;

public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private final RobotContainer robotContainer;

  private final Timer canInitialErrorTimer = new Timer();
  private final Timer canErrorTimer = new Timer();
  private final Timer canivoreErrorTimer = new Timer();
  private final Timer disabledTimer = new Timer();

  private MinoCANBus canivoreBus;
  private MinoCANBus rioBus;

  private final Alert canErrorAlert = new Alert("CAN errors detected, robot may not be controllable.", AlertType.kError);
  private final Alert canivoreErrorAlert = new Alert("CANivore errors detected, robot may not be controllable.", AlertType.kError);
  private final Alert lowBatteryAlert = new Alert("Battery voltage is very low, consider turning off the robot or replacing the battery.", AlertType.kWarning);
  private final Alert jitAlert = new Alert("Please wait to enable, JITing in progress.", AlertType.kInfo);

  public static final BatteryLogger batteryLogger = new BatteryLogger();
  private final BatteryIOInputsAutoLogged batteryInputs = new BatteryIOInputsAutoLogged();

  public Robot() {
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata(
      "GitDirty",
      switch(BuildConstants.DIRTY) {
        case 0 -> "All changes committed";
        case 1 -> "Uncommitted changes";
        default -> "Unknown";
    });

    switch(Constants.getMode()) {
      case REAL:
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());

        break;
      case SIM:
        Logger.addDataReceiver(new NT4Publisher());
        break;
      case REPLAY:
        setUseTiming(false);
        String path = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(path));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(path, "_sim")));
        break;
    }

    SignalLogger.enableAutoLogging(false);
    LiveWindow.disableAllTelemetry();
    Logger.start();

    DriverStation.silenceJoystickConnectionWarning(true);

    RoboRioSim.setTeamNumber(1369);
    if (Constants.getMode() == Constants.Mode.SIM) {
      DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
      DriverStationSim.notifyNewData();
    }

    // Reset alert timers
    canInitialErrorTimer.restart();
    canErrorTimer.restart();
    canivoreErrorTimer.restart();
    disabledTimer.restart();

    // Configure brownout voltage
    RobotController.setBrownoutVoltage(6.0);
    robotContainer = new RobotContainer();

    canivoreBus = new MinoCANBus("*");
    rioBus = new MinoCANBus("rio");
  }

  @Override
  public void robotPeriodic() {
    LoggedTracer.reset();

    PhoenixUtility.refreshAll();
    LoggedTracer.record("PhoenixRefresh");

    batteryInputs.batteryVoltage = RobotController.getBatteryVoltage();
    batteryInputs.rioCurrent = RobotController.getInputCurrent();
    Logger.processInputs("BatteryLogger", batteryInputs);

    batteryLogger.setBatteryVoltage(batteryInputs.batteryVoltage);
    batteryLogger.setRioCurrent(batteryInputs.rioCurrent);
    LoggedTracer.record("BatteryLogger/Periodic");

    if (RobotController.getBatteryVoltage() <= Constants.kLowBatteryVoltage && disabledTimer.hasElapsed(Constants.kLowBatteryDisabledTime)) {
      lowBatteryAlert.set(true);
    }

    CommandScheduler.getInstance().run();
    LoggedTracer.record("Commands");

    var canStatus = RobotController.getCANStatus();
    Logger.recordOutput("CANStatus/OffCount", canStatus.busOffCount);
    Logger.recordOutput("CANStatus/TxFullCount", canStatus.txFullCount);
    Logger.recordOutput("CANStatus/ReceiveErrorCount", canStatus.receiveErrorCount);
    Logger.recordOutput("CANStatus/TransmitErrorCount", canStatus.transmitErrorCount);

    if (canStatus.transmitErrorCount > 0 || canStatus.receiveErrorCount > 0) {
      canErrorTimer.restart();
    }

    canErrorAlert.set(!canErrorTimer.hasElapsed(Constants.kCANivoreErrorTimeThreshold) && canInitialErrorTimer.hasElapsed(Constants.kRioCANErrorTimeThreshold));

    if (Constants.getMode() == Constants.Mode.REAL) {
      var canivoreStatus = this.canivoreBus.getParent().getStatus();
      Logger.recordOutput("CANivoreStatus/Status", canivoreStatus.Status.getName());
      Logger.recordOutput("CANivoreStatus/Utilization", canivoreStatus.BusUtilization);
      Logger.recordOutput("CANivoreStatus/OffCount", canivoreStatus.BusOffCount);
      Logger.recordOutput("CANivoreStatus/TxFullCount", canivoreStatus.TxFullCount);
      Logger.recordOutput("CANivoreStatus/ReceiveErrorCount", canivoreStatus.REC);
      Logger.recordOutput("CANivoreStatus/TransmitErrorCount", canivoreStatus.TEC);

      if (!canivoreStatus.Status.isOK() || canivoreStatus.REC > 0 || canivoreStatus.TEC > 0) {
        canivoreErrorTimer.restart();
      }

      canivoreErrorAlert.set(!canivoreErrorTimer.hasElapsed(Constants.kCANivoreErrorTimeThreshold) && canInitialErrorTimer.hasElapsed(Constants.kRioCANErrorTimeThreshold));
    }

    canivoreBus.updateInputs();
    rioBus.updateInputs();

    if (DriverStation.isEnabled()) {
      disabledTimer.reset();
    }

    robotContainer.getRobotState().updateLogger();

    jitAlert.set(isJITing());
    LoggedTracer.record("RobotPeriodic");
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  public static boolean isJITing() {
    return Timer.getTimestamp() < 45.0;
  }

  public static boolean showHardwareAlerts() {
    return Constants.getMode() != Constants.Mode.SIM;
  }
}
