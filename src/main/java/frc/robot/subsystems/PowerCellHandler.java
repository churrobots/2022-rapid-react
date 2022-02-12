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

public class PowerCellHandler extends SubsystemBase {

  PWMVictorSPX leftIntakeMotor = new PWMVictorSPX(Constants.leftIntakeMotorPWM);
  PWMVictorSPX rightIntakeMotor = new PWMVictorSPX(Constants.rightIntakeMotorPWM);

  public void runPowercellMotor(double leftSpeed, double rightSpeed) {
    leftIntakeMotor.set(-1 * leftSpeed);
    rightIntakeMotor.set(rightSpeed);
  }
}
