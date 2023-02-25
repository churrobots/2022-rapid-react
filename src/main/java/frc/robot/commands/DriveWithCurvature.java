/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.helpers.Gamepad;
import frc.robot.helpers.Gamepad.Axis;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class DriveWithCurvature extends CommandBase {

  protected final Drivetrain drivetrainSubsystem;
  protected final Gamepad controller;

  public DriveWithCurvature(Drivetrain drivetrainSubsystem, Gamepad controller) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.controller = controller;
    this.addRequirements(this.drivetrainSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double throttlePercentage = -1 * controller.leftXAxis.get() * Math.abs(controller.leftXAxis.get());
    double curvaturePercentage = controller.rightXAxis.get() * Math.abs(controller.rightXAxis.get());
    ;
    boolean allowSpinning = true;
    this.drivetrainSubsystem.driveWithCurvature(throttlePercentage, curvaturePercentage, allowSpinning);
  }

  @Override
  public void end(boolean interrupted) {
    this.drivetrainSubsystem.stopDriving();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
