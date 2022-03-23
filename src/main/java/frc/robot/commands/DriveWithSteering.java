/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Tunables;
import frc.robot.helpers.Gamepad.Axis;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class DriveWithSteering extends CommandBase {

  protected final Drivetrain drivetrainSubsystem;
  protected final Axis leftAxis;
  protected final Axis rightAxis;
  protected final Axis rightHorizontalAxis;

  public DriveWithSteering(Drivetrain drivetrainSubsystem, Axis leftAxis, Axis rightAxis, Axis rightHorizontalAxis) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.leftAxis = leftAxis;
    this.rightAxis = rightAxis;
    this.rightHorizontalAxis = rightHorizontalAxis;
    this.addRequirements(this.drivetrainSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double throttleMetersPerSecond = -1 * Tunables.maxDriveMetersPerSecond.get() * leftAxis.get();
    double steeringRotationRadiansPerSecond =  Tunables.maxSteeringRadiansPerSecond.get() * rightHorizontalAxis.get();
    this.drivetrainSubsystem.driveWithThrottleAndSteering(throttleMetersPerSecond, steeringRotationRadiansPerSecond);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
