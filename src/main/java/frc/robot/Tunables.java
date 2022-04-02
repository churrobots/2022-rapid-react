package frc.robot;

import frc.robot.helpers.Tuner.TunableDouble;
import frc.robot.helpers.Tuner.TunableInteger;

public final class Tunables {

  public static final TunableDouble armKF = new TunableDouble("armKF", 0.2);
  public static final TunableDouble armKP = new TunableDouble("armKP", 0.2);
  public static final TunableInteger armSmoothingStrength = new TunableInteger("armSmoothingStrength", 4);
  public static final TunableInteger armCruiseVelocityInSensorUnits = new TunableInteger("armCruiseVelocityInSensorUnits", 16000);
  public static final TunableInteger armAccelerationInSensorUnits = new TunableInteger("armAccelerationInSensorUnits", 7000);

  public static final TunableDouble maxDriveVoltage = new TunableDouble("maxDriveVoltage", 0.80);
  public static final TunableDouble maxDriveAcceleration = new TunableDouble("maxDriveAcceleration", 2.3);

  public static final TunableDouble ejectionSpeedPercentage = new TunableDouble("ejectionSpeedPercentage", -0.9);  

  public static final TunableInteger armUpSensorCounts = new TunableInteger("armUpSensorCounts", 500);
  public static final TunableInteger armDownSensorCounts = new TunableInteger("armDownSensorCounts", -43000);

  public static final TunableDouble climbingDriveVoltage = new TunableDouble("climbingDriveVoltage", 0.55);
  public static final TunableInteger armClimbUnderBarSensorCounts = new TunableInteger("armClimbUnderBarSensorCounts", -33000);
  public static final TunableDouble maxPitchForClimb = new TunableDouble("maxPitchForClimb", 22.0);

  public static final TunableDouble maxArmCurrent = new TunableDouble("maxArmCurrent", 15.0);
  public static final TunableDouble maxArmSeconds = new TunableDouble("maxArmSeconds", 3.0);

}

