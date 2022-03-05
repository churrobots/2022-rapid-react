// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeRight extends SubsystemBase {
  private final PWMVictorSPX rightRollerMotor = new PWMVictorSPX(Constants.rightRollerMotorPWM);
  /** Creates a new Intake. */
  public IntakeRight() {
    rightRollerMotor.setInverted(Constants.rightRollerMotorIsInverted);

  }
  
  public void collectballs() {
    this.rightRollerMotor.set(0.5);

  }

  public void rightejection() {
    this.rightRollerMotor.set(-0.5);

  }

  public void rightstopRollers() {
    this.rightRollerMotor.set(0.0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
