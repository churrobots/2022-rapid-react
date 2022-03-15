/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. Do not put anything functional here.
 */
public final class Constants {

  public static final int falconRearLeftCAN = 1;
  public static final int falconFrontLeftCAN = 4;
  public static final int falconRearRightCAN = 2;
  public static final int falconFrontRightCAN = 3;
  public static final int pigeonCAN = 5;
  public static final int falconArmCAN = 6;
  public static final int leftRollerMotorCAN= 7;
  public static final int rightRollerMotorCAN = 8;

  public static final boolean leftFalconsAreInverted = true;
  public static final boolean rightFalconsAreInverted = false;

  public static final boolean leftRollerMotorIsInverted = true;
  public static final boolean rightRollerMotorIsInverted = false;
  public static final double collectionSpeedPercentage = 0.75;
  public static final double ejectionSpeedPercentage = -0.75;

  public static final int maxSpeedInMetersPerSecond = 3;

  public static final int armSensorDIO = 9;

  public static final double slewRateForDrivetrain = 8.0;


  // The Falcon 500s have a Talon FX Integrated sensor, which is rated for 2048 units per rotation:
  // https://docs.ctre-phoenix.com/en/latest/ch14_MCSensor.html#sensor-resolution
  public static final int sensorUnitsPerRevolution = 2048;
  public static final double driveWheelRadiusInInches = 3;
  public static final double trackWidthInInches = 22.0;
  public static final double drivetrainNeutralDeadbandPercentage = 0.08;
  public static final int armUpSensorCounts = 500;
  public static final int armDownSensorCounts = -41000;
  public static final double armCruiseVelocityInSensorUnits = 15000;
  public static final double armAccelerationInSensorUnits = 6000;
  public static final int armSmoothingStrength = 6;


  // // From the System Identification for Connie
  // public static final double driveGearRatio = 7.31;
  // public static final double kS = 0.62331;
  // public static final double kV = 1.5593;
  // public static final double kA = 0.20183;
  // public static final double kP = 0.021079;
  // public static final double kD = 0.0;

  // From the System Identification for Ponyo
  public static final double driveGearRatio = 8.45;
  public static final double kS = 0.693;
  public static final double kV = 0.748;
  public static final double kA = 0.118;
  public static final double kP = 2.667;
  public static final double kD = 0.0;

  public static final int driverGamepadPort = 0;
  public static final int operatorGamepadPort = 1;

}
