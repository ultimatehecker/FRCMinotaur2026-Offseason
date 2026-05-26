package frc.minolib.energy;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import frc.minolib.advantagekit.LoggedTracer;
import frc.minolib.io.BatteryIOInputsAutoLogged;
import frc.minolib.utilities.FullSubsystem;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.GlobalConstants;

import static edu.wpi.first.units.Units.Amps;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.google.flatbuffers.Constants;

public class EnergyManagement extends FullSubsystem {
  private static final double minVoltageBrownout = 7.0;
  private static final double maxBudgetAmps = 200.0;
  private static final double breakerNiceness = 0.05;
  private static final double budgetWarningThreshold = 180.0;
  private static final double configPeriodSeconds = 0.1;
  private static final double budgetHeadroom = 0.9;
  private static final double breakerDangerHorizonSecs = 3.0;
  private final double breakerDamageWarningThreshold;

  private static final Alert budgetWarning = new Alert("Battery is low, robot performance may be degraded.", AlertType.kInfo);
  private static final Debouncer budgetWarningDebouncer = new Debouncer(0.5, DebounceType.kBoth);
  private static final Alert brownoutWarning = new Alert("Brownout detected, drive performance may be degraded.", AlertType.kWarning);
  private static final Alert breakerDamageWarning = new Alert("Breaker damage is high, please stop using the robot.", AlertType.kWarning);
  private static final Debouncer breakerDamageWarningDebouncer = new Debouncer(0.5, DebounceType.kBoth);

  private static EnergyManagement instance;

  public static EnergyManagement getInstance() {
    if (instance == null) instance = new EnergyManagement();
    return instance;
  }

  private final BatteryLogger energyLogger = new BatteryLogger();
  private final BatteryEstimator battery = new BatteryEstimator();
  private final Breaker breaker = new Breaker(breakerNiceness);
  private final BatteryIOInputsAutoLogged inputs = new BatteryIOInputsAutoLogged();

  private double budget = 0.0;
  private double driveBudget = 0.0;
  private Debouncer brownoutDebouncer = new Debouncer(2.0, DebounceType.kFalling);

  private EnergyManagement() {
    // Solve for damage state where breaker will trip if we run at maxBudgetAmps for horizon.
    breakerDamageWarningThreshold = (1.0 - breakerNiceness) - (breakerDangerHorizonSecs / Breaker.getTripTime(maxBudgetAmps / Breaker.I_RATED));
  }

  public void reset() {
    battery.setInitialVoltage(inputs.batteryVoltage, energyLogger.getTotalCurrent());
  }

  public void reportCurrentUsage(String key, boolean drive, double... amps) {
    double totalAmps = 0.0;
    for (double amp : amps) totalAmps += Math.max(0.0, amp);
    energyLogger.reportCurrentUsage(key, drive, totalAmps);
  }

  @Override
  public void periodic() {
    inputs.batteryVoltage = RobotController.getBatteryVoltage();
    inputs.rioCurrent = RobotController.getInputCurrent();
    inputs.brownedOut = RobotController.isBrownedOut();
    Logger.processInputs("EnergyLogger", inputs);
    energyLogger.setBatteryVoltage(inputs.batteryVoltage);
    energyLogger.setRioCurrent(inputs.rioCurrent);
    LoggedTracer.record("EnergyManager/Periodic");
  }

  @Override
  public void periodicAfterScheduler() {
    // Run energy logger
    energyLogger.periodicAfterScheduler();

    // Update models
    battery.update(energyLogger.getTotalCurrent(), inputs.batteryVoltage);
    breaker.update(energyLogger.getTotalCurrent());

    // Calculate budgets
    double batteryMaxCurrent = battery.calculateMaxCurrent(minVoltageBrownout, configPeriodSeconds);
    double breakerMaxCurrent = breaker.calculateMaxCurrent(breakerDangerHorizonSecs);
    Logger.recordOutput("EnergyManager/BatteryMaxCurrent", batteryMaxCurrent);
    Logger.recordOutput("EnergyManager/BreakerMaxCurrent", breakerMaxCurrent);

    budget = Math.min(Math.min(batteryMaxCurrent, breakerMaxCurrent) * budgetHeadroom, maxBudgetAmps);

    boolean brownoutDebounced = brownoutDebouncer.calculate(inputs.brownedOut);
    if (!brownoutDebounced) {
      driveBudget = budget - energyLogger.getTotalCurrent() + energyLogger.getDriveCurrent();
    } else {
      double calculatedBudget = budget - energyLogger.getTotalCurrent() + energyLogger.getDriveCurrent();
      driveBudget = calculatedBudget < driveBudget ? calculatedBudget : Math.min(calculatedBudget, driveBudget + DrivetrainConstants.kDriveProbeRateBrownout.magnitude() * GlobalConstants.kLoopPeriodSeconds); // Asymmetric ramping of drive budget
    }

    driveBudget = Math.max(0.0, driveBudget);

    Logger.recordOutput("EnergyManager/Budget", budget);
    Logger.recordOutput("EnergyManager/DriveBudget", driveBudget);

    // Update alerts
    budgetWarning.set(budgetWarningDebouncer.calculate(budget < budgetWarningThreshold));
    brownoutWarning.set(brownoutDebounced);
    breakerDamageWarning.set(breakerDamageWarningDebouncer.calculate(breaker.getDamageState() > breakerDamageWarningThreshold));

    energyLogger.resetTotals();
    LoggedTracer.record("EnergyManager/AfterScheduler");
  }

  /** Get the current limits for a subsystem. */
  public double getDriveCurrentLimit() {
    double driveLimit = Math.floor(MathUtil.clamp(driveBudget / 4.0, DrivetrainConstants.kDriveMinimumSupplyCurrentLimit.in(Amps), DrivetrainConstants.kDriveMaximumSupplyCurrentLimit.in(Amps)) / 0.5) * 0.5;

    if (DriverStation.isAutonomous()) {
      driveLimit = Amps.of(50).magnitude();
    }

    Logger.recordOutput("EnergyManager/DriveLimit", driveLimit);
    return driveLimit;
  }

  public double getBatteryVoltage() {
    return inputs.batteryVoltage;
  }
}