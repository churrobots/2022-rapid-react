// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.LinkedList;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Tunables;
import frc.robot.helpers.Tuner.TunableInteger;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;

public class AutoClimb extends CommandBase {
  protected final Arm armSubsystem;
  protected final Drivetrain drivetrain;
  protected LinkedList<Double> pitchReadings = new LinkedList<Double>();

  protected final TunableInteger armDownSensorCounts = new TunableInteger("armDownSensorCounts",
      Tunables.armDownSensorCounts.get());
  
  public AutoClimb(Arm armsubsystem, Drivetrain drivetrain) {
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
    armSubsystem.moveToPosition(-22000);
    drivetrain.driveWithMetersPerSecond(2, 2);
    double pitch_value = drivetrain.getPitch();
    if (pitch_value > 26) {
      drivetrain.driveWithPercentages(0, 0);
      armSubsystem.moveToPosition(Tunables.armDownSensorCounts.get());
    }

    // FInd way to check pitch value
    // will stop the motor, potentially move the arm foward or back
    // to level the bot well


      
    }
    
    
  
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
