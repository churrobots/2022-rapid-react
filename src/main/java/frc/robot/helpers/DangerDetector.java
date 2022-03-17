// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.helpers;

import java.util.LinkedList;

public class DangerDetector {

  private LinkedList<Double> pitchReadings = new LinkedList<Double>();
  private LinkedList<Double> rollReadings = new LinkedList<Double>();

  // TODO: make this configurable by the caller
  private int windowSize = 50;
  private double minValue = 5.0;

  public void update(double pitch, double roll) {
    pitchReadings.addLast(pitch);
    rollReadings.addLast(roll);
    if (pitchReadings.size() > windowSize) {
      pitchReadings.removeFirst();
    }
    if (rollReadings.size() > windowSize) {
      rollReadings.removeFirst();
    }
  }

  public double getDangerLevel() {
    double sumOfPitches = 0;
    double sumOfRolls = 0;
    var pitchIterator = pitchReadings.iterator();
    var rollIterator = rollReadings.iterator();
    while (pitchIterator.hasNext()) {
      double nextValue = Math.abs(pitchIterator.next());
      if (nextValue < minValue) {
        nextValue = 0;
      }
      sumOfPitches += nextValue;
    }
    while (rollIterator.hasNext()) {
      double nextValue = Math.abs(rollIterator.next());
      if (nextValue < minValue) {
        nextValue = 0;
      }
      sumOfRolls += nextValue;
    }
    return sumOfRolls + sumOfPitches;
  }

}
