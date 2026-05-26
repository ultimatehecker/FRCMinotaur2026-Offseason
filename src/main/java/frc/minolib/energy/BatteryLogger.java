package frc.minolib.energy;

import java.util.HashMap;
import java.util.Map;
import lombok.Getter;
import lombok.Setter;

import org.littletonrobotics.junction.Logger;

import frc.robot.constants.GlobalConstants;

public class BatteryLogger {
    private double totalCurrent = 0.0;
    @Getter private double driveCurrent = 0.0;
    private double totalPower = 0.0;
    private double totalEnergy = 0.0;

    @Setter private double batteryVoltage = 12.6;
    @Setter private double rioCurrent = 0.0;

    private Map<String, Double> subsytemCurrents = new HashMap<>();
    private Map<String, Double> subsytemPowers = new HashMap<>();
    private Map<String, Double> subsytemEnergies = new HashMap<>();

    public void reportCurrentUsage(String key, boolean drive, double... amps) {
        double totalAmps = 0.0;
        for (double amp : amps) totalAmps += Math.abs(amp);

        if (drive) {
            driveCurrent += totalAmps;
        }

        double power = totalAmps * batteryVoltage;
        double energy = power * GlobalConstants.kLoopPeriodSeconds;

        totalCurrent += totalAmps;
        totalPower += power;
        totalEnergy += energy;

        subsytemCurrents.put(key, totalAmps);
        subsytemPowers.put(key, power);
        subsytemEnergies.merge(key, energy, Double::sum);

        String[] keys = key.split("/|-");
        if (keys.length < 2) {
            return;
        }

        String subkey = "";
        for (int i = 0; i < keys.length - 1; i++) {
            subkey += keys[i];
            if (i < keys.length - 2) {
                subkey += "/";
            }

            subsytemCurrents.merge(subkey, totalAmps, Double::sum);
            subsytemPowers.merge(subkey, power, Double::sum);
            subsytemEnergies.merge(subkey, energy, Double::sum);
        }
    }

    public void periodicAfterScheduler() {
        reportCurrentUsage("EnergyLogger/Controls/roboRIO", false, rioCurrent);
        reportCurrentUsage("EnergyLogger/Controls/CANcoders", false, 0.05 * 4);
        reportCurrentUsage("EnergyLogger/Controls/Pigeon", false, 0.04);
        reportCurrentUsage("EnergyLogger/Controls/CANivore", false, 0.03);
        reportCurrentUsage("EnergyLogger/Controls/Radio", false, 0.5);

        // Log total and subsystem energy usage
        Logger.recordOutput("EnergyLogger/Current", totalCurrent, "amps");
        Logger.recordOutput("EnergyLogger/Power", totalPower, "watts");
        Logger.recordOutput("EnergyLogger/Energy", joulesToWattHours(totalEnergy), "watt hours");

        for (var entry : subsytemCurrents.entrySet()) {
            Logger.recordOutput("EnergyLogger/Current/" + entry.getKey(), entry.getValue(), "amps");
            subsytemCurrents.put(entry.getKey(), 0.0);
        }

        for (var entry : subsytemPowers.entrySet()) {
            Logger.recordOutput("EnergyLogger/Power/" + entry.getKey(), entry.getValue(), "watts");
            subsytemPowers.put(entry.getKey(), 0.0);
        }

        for (var entry : subsytemEnergies.entrySet()) {
            Logger.recordOutput("EnergyLogger/Energy/" + entry.getKey(), joulesToWattHours(entry.getValue()), "watt hours");
        }
    }

    public void resetTotals() {
        totalPower = 0.0;
        totalCurrent = 0.0;
        driveCurrent = 0.0;
    }

    public double getTotalCurrent() {
        return totalCurrent;
    }

    public double getTotalPower() {
        return totalPower;
    }

    public double getTotalEnergy() {
        return totalEnergy;
    }

    private double joulesToWattHours(double joules) {
        return joules / 3600.0;
    }
}