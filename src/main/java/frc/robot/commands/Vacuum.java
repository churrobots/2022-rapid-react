// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeLeft;
import frc.robot.subsystems.IntakeRight;

public class Vacuum extends CommandBase {
  protected final IntakeLeft polterLeftGust3000;
  protected final IntakeRight polterRightGust3000;
  /** Creates a new Vacuum. */

  public Vacuum(IntakeLeft polterLeftGust3000, IntakeRight polterRightGust3000) {
    this.polterLeftGust3000 = polterLeftGust3000;
    this.polterRightGust3000 = polterRightGust3000;

    this.addRequirements(this.polterLeftGust3000);
    this.addRequirements(this.polterRightGust3000);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    polterLeftGust3000.collectballs();
    polterRightGust3000.collectballs();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    polterRightGust3000.rightstopRollers();
    polterLeftGust3000.leftstopRollers();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  } 
}
