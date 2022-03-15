// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.helpers;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Add your docs here. */
public class SubsystemTuner {

  protected static class TunableEntry<T> {
    protected final T defaultValue;
    protected final NetworkTableEntry entry;
    protected long lastDetectedChange;

    public TunableEntry(String subsystemName, String name, T defaultValue) {
      NetworkTable table = NetworkTableInstance.getDefault().getTable(subsystemName).getSubTable("tuner");
      entry = table.getEntry(name);
      this.defaultValue = defaultValue;
      lastDetectedChange = entry.getLastChange();
    }
  }

  public static class TunableBoolean extends TunableEntry<Boolean> {
    public TunableBoolean(String subsystemName, String name, Boolean defaultValue) {
      super(subsystemName, name, defaultValue);
      entry.setBoolean(defaultValue);
    }
    public Boolean get() {
      return entry.getBoolean(defaultValue);
    }
  }

  public static class TunableDouble extends TunableEntry<Double> {
    public TunableDouble(String subsystemName, String name, Double defaultValue) {
      super(subsystemName, name, defaultValue);
      entry.setDouble(defaultValue);
    }

    public Double get() {
      return entry.getDouble(defaultValue);
    }

    public boolean didChange() {
      boolean answer = lastDetectedChange != entry.getLastChange();
      lastDetectedChange = entry.getLastChange();
      return answer;
    }
  }

}
