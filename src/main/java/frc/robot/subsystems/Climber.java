// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import org.ejml.equation.Variable;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Tunables;
import frc.robot.helpers.SubsystemInspector;

public class Climber extends SubsystemBase {
  
  private final WPI_TalonFX climberMotor = new WPI_TalonFX(Constants.falconClimberCAN);

  private final SubsystemInspector inspector = new SubsystemInspector("Climber");

   public Climber() {
    climberMotor.configFactoryDefault();
    climberMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
  }

  private double getCurrentPosition() {
    return climberMotor.getSelectedSensorPosition();
  }

  @Override
  public void periodic() {
    if (RobotState.isDisabled()) {
      climberMotor.setSelectedSensorPosition(0);
      climberMotor.setNeutralMode(NeutralMode.Coast);
    } else {
      climberMotor.setNeutralMode(NeutralMode.Brake);
    }
    inspector.set("currentPosition", getCurrentPosition());
  }

  public void moveUp() {
    var hitUpperLimit = getCurrentPosition() < -228000;
    if (!hitUpperLimit) {
      climberMotor.setVoltage(-0.5);
    }
  }

  public void moveDown() {
    var hitlowerlimit = getCurrentPosition() > -4000;
    if (!hitlowerlimit) {
      climberMotor.setVoltage(0.5);
    }
  }

  public void stop() {
  climberMotor.setVoltage(0);
}
}
