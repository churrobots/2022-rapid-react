/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.commands.DriveAsTank;
import frc.robot.commands.EjectLeft;
import frc.robot.commands.EjectRight;
import frc.robot.commands.MoveArmDown;
import frc.robot.commands.MoveArmUp;
import frc.robot.commands.ResetDrivetrainSensors;
import frc.robot.commands.Vacuum;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IntakeLeft;
import frc.robot.subsystems.IntakeRight;
import frc.robot.commands.AutoDriveOffTarmac;
import frc.robot.helpers.Gamepad;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  public final Command driveOffTarmac;

  public RobotContainer() {

    // Default select the Tuner tab
    // Shuffleboard.selectTab("game");

    // Connect to all the inputs (gamepads and shuffleboard).
    Gamepad driverGamepad = new Gamepad(Constants.driverGamepadPort); 
    Gamepad operatorGamepad = new Gamepad(Constants.operatorGamepadPort);

    // Connect to all the outputs.
    Drivetrain drivetrain = new Drivetrain();
    IntakeLeft polterLeftGust3000 = new IntakeLeft();
    IntakeRight polterRightGust3000 = new IntakeRight();
    Arm muscleArm = new Arm();

    // Describe when the commands should be scheduled.
    this.driveOffTarmac = new AutoDriveOffTarmac(drivetrain);

    drivetrain.setDefaultCommand(new DriveAsTank(drivetrain, driverGamepad.leftYAxis, driverGamepad.rightYAxis,
        driverGamepad.rightAnalogTrigger));
    operatorGamepad.bButton.whenHeld(new Vacuum(polterLeftGust3000, polterRightGust3000));
    operatorGamepad.leftBumper.whenHeld(new EjectLeft(polterLeftGust3000));
    operatorGamepad.rightBumper.whenHeld(new EjectRight(polterRightGust3000));
    operatorGamepad.povUp.whenHeld(new MoveArmUp(muscleArm));
    operatorGamepad.povDown.whenHeld(new MoveArmDown(muscleArm));
  }

  public Command getAutonomousCommand() {
    // TODO: use the value from the Station chooser
    // TODO: figure out how to make this reset the encoders onclick in the Station dropdown
    // TODO: figureo out how to make this initialize the Pose correctly onclick (for auto choices)
    return this.driveOffTarmac;
  }

}
