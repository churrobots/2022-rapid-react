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

public class LowerArmDown extends CommandBase {

  Arms arms;
  double startTime;
  double millisecondsUntilDown;

  public LowerArmDown(Arms arms) {
    addRequirements(arms);
    this.arms = arms;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.startTime = System.currentTimeMillis();
    this.millisecondsUntilDown = Tuner.getSecondsToMoveArm() * 1000;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double elapsedMilliseconds = System.currentTimeMillis() - this.startTime;
    if (elapsedMilliseconds > this.millisecondsUntilDown) {
      this.arms.holdDown();
    } else {
      this.arms.moveDown();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.arms.holdDown();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
