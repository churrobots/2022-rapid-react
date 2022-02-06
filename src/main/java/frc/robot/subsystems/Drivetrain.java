/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {

  protected DifferentialDrive differentialDrive;

  public Drivetrain() {

    WPI_TalonSRX driveRearLeftMotor = new WPI_TalonSRX(Constants.driveRearLeftCAN);
    WPI_TalonSRX driveFrontLeftMotor = new WPI_TalonSRX(Constants.driveFrontLeftCAN);
    WPI_TalonSRX driveRearRightMotor = new WPI_TalonSRX(Constants.driveRearRightCAN);
    WPI_TalonSRX driveFrontRightMotor = new WPI_TalonSRX(Constants.driveFrontRightCAN);

    MotorControllerGroup leftMotors = new MotorControllerGroup(driveFrontLeftMotor, driveRearLeftMotor);
    MotorControllerGroup rightMotors = new MotorControllerGroup(driveFrontRightMotor, driveRearRightMotor);
    leftMotors.setInverted(true);
    rightMotors.setInverted(false);
    this.differentialDrive = new DifferentialDrive(leftMotors, rightMotors);
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    // TODO: consider filtering for smoother joystick driving
    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/filters/slew-rate-limiter.html
    // TODO: also could just try using the version of tankDrive that takes a 3rd argument true to decrease sensitivity at low speeds
    this.differentialDrive.tankDrive(leftSpeed, rightSpeed);
  }

}
