// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  
  private final WPI_TalonFX armMotor = new WPI_TalonFX(Constants.falconArmCAN);
  /** Creates a new Arm. */
  public Arm() {
    armMotor.configFactoryDefault();
    armMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    // armMotor.set(TalonFXControlMode.Position, 1000);
  }
  
  public void moveUp() {
    this.armMotor.set(0.5);

  }
  
  public void moveDown() {
    this.armMotor.set(-0.5);
  }

  public void stopArm() {
    this.armMotor.set(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
