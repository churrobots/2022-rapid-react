/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Tuner;
import frc.robot.subsystems.Arms;

public class RaiseArmUp extends CommandBase {

  Arms arms;
  double startTime;
  double millisecondsUntilUp = Tuner.getSecondsToMoveArm() * 1000;

  public RaiseArmUp(Arms arms) {
    addRequirements(arms);
    this.arms = arms;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.startTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double elapsedMilliseconds = System.currentTimeMillis() - this.startTime;
    if (elapsedMilliseconds > this.millisecondsUntilUp) {
      this.arms.holdUp();
    } else {
      this.arms.moveUp();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.arms.holdUp();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
