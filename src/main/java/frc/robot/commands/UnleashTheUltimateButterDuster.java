// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ButterDuster;

public class UnleashTheUltimateButterDuster extends CommandBase {
  private ButterDuster butterDuster;
  private Timer timer = new Timer();
  /** Creates a new UnleashTheUltimateButterDuster. */
  public UnleashTheUltimateButterDuster(
      ButterDuster butterDuster
    
  ) {
    this.butterDuster = butterDuster;
    this.addRequirements(butterDuster);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    butterDuster.unleash();
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.get() > 0.25) {
      butterDuster.contain();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    butterDuster.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > 0.9;
  }
}
