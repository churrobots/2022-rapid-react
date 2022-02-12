/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {

  TalonSRX driveRearLeftMotor = new TalonSRX(Constants.driveRearLeftCAN);
  TalonSRX driveFrontLeftMotor = new TalonSRX(Constants.driveFrontLeftCAN);

  TalonSRX driveRearRightMotor = new TalonSRX(Constants.driveRearRightCAN);
  TalonSRX driveFrontRightMotor = new TalonSRX(Constants.driveFrontRightCAN);

  public Drivetrain() {

    driveRearLeftMotor.setInverted(Constants.leftIsInverted);
    driveFrontLeftMotor.setInverted(Constants.leftIsInverted);
    driveRearLeftMotor.follow(driveFrontLeftMotor);

    driveRearRightMotor.setInverted(Constants.rightIsInverted);
    driveFrontRightMotor.setInverted(Constants.rightIsInverted);
    driveRearRightMotor.follow(driveFrontRightMotor);

  }

  public void tankDrive(double leftPercent, double rightPercent) {
    // TODO: consider filtering for smoother joystick driving
    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/filters/slew-rate-limiter.html
    // TODO: also could just try using the version of tankDrive that takes a 3rd argument true to decrease sensitivity at low speeds
    driveFrontLeftMotor.set(TalonSRXControlMode.PercentOutput, leftPercent);
    driveFrontRightMotor.set(TalonSRXControlMode.PercentOutput, rightPercent);    
  }

}
