// Copyright (c) FIRST and other WPILib contributors.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.helpers.SubsystemInspector;

public class ButterDuster extends SubsystemBase {
  
  private final WPI_VictorSPX butterDusterMotor = new WPI_VictorSPX(Constants.butterDusterCAN);
  private final Timer timer = new Timer();
  private boolean wasUnleashing = false;

  private final SubsystemInspector inspector = new SubsystemInspector("ButterDuster");
  

  public ButterDuster() {

  }

  @Override
  public void periodic() {
    inspector.set("currentCommand", this.getCurrentCommand());
  }

  public void unleash() {
    if (!wasUnleashing) {
      timer.reset();
      wasUnleashing = true;
    }
    var hasBeenRunningForHalfSecond = timer.get() > 0.5;
    if (hasBeenRunningForHalfSecond) {
      stop();
    } else {
      butterDusterMotor.setVoltage(0.10);
    }
  }

  public void contain() {
    var wasContaining = !wasUnleashing;
    if (wasContaining) {
      timer.reset();
      wasUnleashing = false;
    }
    var hasBeenRunningForQuarterSecond = timer.get() > 0.25;
    if (hasBeenRunningForQuarterSecond) {
      stop();
    } else {
      butterDusterMotor.setVoltage(-0.10);
    }
  }

  public void stop() {
    butterDusterMotor.setVoltage(0);
  }
}
