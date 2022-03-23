// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Tunables;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;

public class AssistedClimb extends CommandBase {
  protected final Arm armSubsystem;
  protected final Drivetrain drivetrain;
  
  public AssistedClimb(Arm armsubsystem, Drivetrain drivetrain) {
    this.armSubsystem = armsubsystem;
    this.drivetrain = drivetrain;
    this.addRequirements(this.drivetrain);
    this.addRequirements(this.armSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Tuck the arm under the bar
    int desiredArmPosition = Tunables.armClimbUnderBarSensorCounts.get();
    armSubsystem.moveToPosition(desiredArmPosition);

    // Drive forward when it's safe
    double distanceFromDesiredArmPosition = Math.abs(armSubsystem.getCurrentPosition() - desiredArmPosition);
    boolean isSafeToDriveUnder = distanceFromDesiredArmPosition < 300;
    if (isSafeToDriveUnder) {
      drivetrain.driveWithCurvature(Tunables.driveTrainClimbingPercentage.get(), 0, false);
    }

    // Detect when we hang and try to pitch forward when needed.
    double pitch_value = drivetrain.getPitch();
    if (pitch_value > Tunables.maxPitchForClimb.get()) {
      drivetrain.stopDriving();
      armSubsystem.moveToPosition(Tunables.armDownSensorCounts.get());
    }      
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.moveToPosition(Tunables.armDownSensorCounts.get());
    drivetrain.stopDriving();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
