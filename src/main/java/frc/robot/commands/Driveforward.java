/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class Driveforward extends CommandBase {

  Drivetrain drivetrain;
  double startTime;

  public Driveforward(Drivetrain theDrivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(theDrivetrain);
    this.drivetrain = theDrivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.startTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double howlong = System.currentTimeMillis() - this.startTime;
    if (howlong < 2000) {
      this.drivetrain.tankDrive(.5, .5);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.drivetrain.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double howlong = System.currentTimeMillis() - this.startTime;
    if (howlong > 2000) {
      return true;
    } else {
      return false;
    }
  }
}
