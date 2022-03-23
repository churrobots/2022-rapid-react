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
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * An example command that uses an example subsystem.
 */
public class DriveWithCurvature extends CommandBase {

  protected final Drivetrain drivetrainSubsystem;
  protected final Axis leftAxis;
  protected final Axis rightHorizontalAxis;
  protected final JoystickButton spinButton;

  public DriveWithCurvature(Drivetrain drivetrainSubsystem, Axis leftAxis, Axis rightHorizontalAxis, JoystickButton spinButton) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.leftAxis = leftAxis;
    this.spinButton = spinButton;
    this.rightHorizontalAxis = rightHorizontalAxis;
    this.addRequirements(this.drivetrainSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double throttlePercentage = -1 * leftAxis.get();
    double curvaturePercentage = rightHorizontalAxis.get();
    boolean allowSpinning = spinButton.get();
    this.drivetrainSubsystem.driveWithCurvature(throttlePercentage, curvaturePercentage, allowSpinning);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
