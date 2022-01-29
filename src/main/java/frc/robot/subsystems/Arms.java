/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Tuner;

public class Arms extends SubsystemBase {

  PWMVictorSPX armMotor = new PWMVictorSPX(Constants.armMotorPWM);

  public Arms() {}

  public void moveUp() {
    this.armMotor.set(Tuner.getPowerToMoveArm());
  }

  public void moveDown() {
    this.armMotor.set(-1 * Tuner.getPowerToMoveArm());
  }

  public void holdUp() {
    this.armMotor.set(Tuner.getPowerToHoldUpArm());
  }

  public void holdDown() {
    this.armMotor.set(-1 * Tuner.getPowerToHoldDownArm());
  }

}
