/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Tuner;
import frc.robot.helpers.Gamepad.Axis;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class DriveAsTank extends CommandBase {

  protected final Drivetrain drivetrainSubsystem;
  protected final Axis leftAxis;
  protected final Axis rightAxis;
  protected final Axis boostAxis;

  public DriveAsTank(Drivetrain drivetrainSubsystem, Axis leftAxis, Axis rightAxis, Axis boostAxis) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.leftAxis = leftAxis;
    this.rightAxis = rightAxis;
    this.boostAxis = boostAxis;
    this.addRequirements(this.drivetrainSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double normalSpeed = Tuner.getMaxNonBoostDrivePower();
    double maxBoost = 1 - normalSpeed;
    double boost = boostAxis.get() * maxBoost;
    double maxSpeed = normalSpeed + (maxBoost * boost);
    double leftSpeed = this.leftAxis.get() * maxSpeed;
    double rightSpeed = this.rightAxis.get() * maxSpeed;
    this.drivetrainSubsystem.tankDrive(leftSpeed, rightSpeed);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
