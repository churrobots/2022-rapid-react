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
  
  // HACK: just testing out MotionMagic by co-opting the drivetrain right now
  // private final WPI_TalonFX armMotor = new WPI_TalonFX(Constants.falconArmCAN);
  private final WPI_TalonFX armMotor = new WPI_TalonFX(Constants.falconFrontLeftCAN);
  private final DigitalInput armSensor = new DigitalInput(Constants.armSensorDIO);
  private boolean didResetEncoderOnce = false;

  private final SubsystemInspector inspector = new SubsystemInspector("Arm");
  
  /** Creates a new Arm. */
  public Arm() {
    // HACK: using the drivetrain to try out MotionMagic, don't fight the other configs it got
    // armMotor.configFactoryDefault();
    // armMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    // This helper method sets a lot of the defaults that the example code seems to require
    // for full functioning of the Falcon500s.
    // https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/Java%20Talon%20FX%20(Falcon%20500)/MotionMagic/src/main/java/frc/robot/Robot.java
    // TODO: calculate or characterize these values? why would you ever not use the 0th slots?
    int fakeSlot = 0;
    int fakePIDSlot = 0;
    int fakeTimeoutMilliseconds = 30;
    double fakeKP = 0.2;
    double fakeKI = 0.0;
    double fakeKD = 0.0;
    double fakeKF = 0.2;
    armMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, fakeTimeoutMilliseconds);
		armMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, fakeTimeoutMilliseconds);

		armMotor.configNominalOutputForward(0, fakeTimeoutMilliseconds);
		armMotor.configNominalOutputReverse(0, fakeTimeoutMilliseconds);
		armMotor.configPeakOutputForward(1, fakeTimeoutMilliseconds);
		armMotor.configPeakOutputReverse(-1, fakeTimeoutMilliseconds);

		armMotor.selectProfileSlot(fakeSlot, fakePIDSlot);
		armMotor.config_kF(fakeSlot, fakeKF, fakeTimeoutMilliseconds);
		armMotor.config_kP(fakeSlot, fakeKP, fakeTimeoutMilliseconds);
		armMotor.config_kI(fakeSlot, fakeKI, fakeTimeoutMilliseconds);
		armMotor.config_kD(fakeSlot, fakeKD, fakeTimeoutMilliseconds);

		/* Set acceleration and vcruise velocity - see documentation */
		armMotor.configMotionCruiseVelocity(15000, fakeTimeoutMilliseconds);
		armMotor.configMotionAcceleration(5000, fakeTimeoutMilliseconds);
    armMotor.configMotionSCurveStrength(8);

  }

  public void moveToPosition(int sensorCountsFromUpPosition) {
    this.armMotor.set(TalonFXControlMode.MotionMagic, sensorCountsFromUpPosition);
    inspector.set("moveToPosition", sensorCountsFromUpPosition);
  }

  @Override
  public void periodic() {
    if (didReachCalibrationPosition()) {
      armMotor.setSelectedSensorPosition(0);
    } else {
      moveSlowlyIntoCalibrationPosition();
    }
    inspector.set("armSensor", armSensor.get());
    inspector.set("didResetEncoderOnce", didResetEncoderOnce);
  }

  private boolean didReachCalibrationPosition() {
    return !armSensor.get();
  }

  private void moveSlowlyIntoCalibrationPosition() {
    // TODO: look at PDP current draw for the motor as a sanity check
    armMotor.set(0.15);
  }

}
