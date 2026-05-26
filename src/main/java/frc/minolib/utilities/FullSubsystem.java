package frc.minolib.utilities;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class FullSubsystem extends SubsystemBase {
  private static List<FullSubsystem> instances = new ArrayList<>();

  public FullSubsystem() {
    super();
    instances.add(this);
  }

  public FullSubsystem(String name) {
    super(name);
    instances.add(this);
  }

  /** This method is called periodically after the command scheduler, and should be used for applying outputs. */
  public abstract void periodicAfterScheduler();

  /** Run the "after periodic" methods for all subsystems. */
  public static void runAllPeriodicAfterScheduler() {
    for (FullSubsystem instance : instances) {
      instance.periodicAfterScheduler();
    }
  }
}