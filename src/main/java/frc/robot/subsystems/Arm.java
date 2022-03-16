// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.helpers.SubsystemInspector;

public class Arm extends SubsystemBase {
  
  private final WPI_TalonFX armMotor = new WPI_TalonFX(Constants.falconArmCAN);
  private final DigitalInput armSensor = new DigitalInput(Constants.armSensorDIO);
  private boolean isCalibrating = true;

  private final SubsystemInspector inspector = new SubsystemInspector("Arm");

  /** Creates a new Arm. */
  public Arm() {
    armMotor.configFactoryDefault();
    armMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    // This sets a lot of the defaults that the example code seems to require
    // for full functioning of the Falcon500s. Cargo culting FTW.
    // https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/Java%20Talon%20FX%20(Falcon%20500)/MotionMagic/src/main/java/frc/robot/Robot.java
    // TODO: calculate or characterize these values? why would you ever not use the 0th slots?
    int fakeSlot = 0;
    int fakePIDSlot = 0;
    int fakeTimeoutMilliseconds = 30;
    double fakeKP = 0.2;
    double fakeKI = 0.0;
    double fakeKD = 0.0;
    double fakeKF = 0.2;

    armMotor.configNeutralDeadband(0.001); // really low deadzone

    armMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, fakeTimeoutMilliseconds);
		armMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, fakeTimeoutMilliseconds);

		armMotor.selectProfileSlot(fakeSlot, fakePIDSlot);
		armMotor.config_kF(fakeSlot, fakeKF, fakeTimeoutMilliseconds);
		armMotor.config_kP(fakeSlot, fakeKP, fakeTimeoutMilliseconds);
		armMotor.config_kI(fakeSlot, fakeKI, fakeTimeoutMilliseconds);
		armMotor.config_kD(fakeSlot, fakeKD, fakeTimeoutMilliseconds);

		armMotor.configMotionCruiseVelocity(Constants.armCruiseVelocityInSensorUnits, fakeTimeoutMilliseconds);
		armMotor.configMotionAcceleration(Constants.armAccelerationInSensorUnits, fakeTimeoutMilliseconds);
    armMotor.configMotionSCurveStrength(Constants.armSmoothingStrength);
    
    armMotor.configPeakOutputForward(0.4);
    armMotor.configPeakOutputReverse(-0.4);
  }

  public void forceCalibration() {
    isCalibrating = true;
  }

  public void moveToPosition(int sensorCountsFromUpPosition) {
    if(!isCalibrating){
      this.armMotor.set(TalonFXControlMode.MotionMagic, sensorCountsFromUpPosition);
      inspector.set("moveToPosition", sensorCountsFromUpPosition);
    }
  }

  @Override
  public void periodic() {
    calibrateIfNeeded();
    inspector.set("armSensor", armSensor.get());
    inspector.set("isCalibrating", isCalibrating);
    inspector.set("currentCommand", this.getCurrentCommand());
    inspector.set("sensorCount", this.armMotor.getSelectedSensorPosition());
    inspector.set("currentCommand", this.getCurrentCommand());
    inspector.set("power:supplyCurrent", armMotor.getSupplyCurrent());
    inspector.set("power:statorCurrent", armMotor.getStatorCurrent());
  }

  private void calibrateIfNeeded() {
    final boolean isArmUp = armSensor.get() == true;
    if (isCalibrating) {
      if (isArmUp) {
        armMotor.set(TalonFXControlMode.PercentOutput, 0);
        armMotor.setSelectedSensorPosition(0);
        isCalibrating = false; 
      }
      else {
        armMotor.set(TalonFXControlMode.PercentOutput, Constants.armCalibrationSpeedPercentage);
      }
    }
  }

}
