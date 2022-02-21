/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.commands.DriveAsTank;
import frc.robot.subsystems.Drivetrain;
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

  public final Command autonomousCommand;

  public RobotContainer() {

    // Default select the Tuner tab
    // Shuffleboard.selectTab("game");

    // Connect to all the inputs (gamepads and shuffleboard).
    Gamepad driverGamepad = new Gamepad(Constants.driverGamepadPort); 
    Gamepad operatorGamepad = new Gamepad(Constants.operatorGamepadPort);

    // Connect to all the outputs.
    Drivetrain drivetrain = new Drivetrain();

    // Describe when the commands should be scheduled.
    this.autonomousCommand = new AutoDriveOffTarmac(drivetrain);

    drivetrain.setDefaultCommand(new DriveAsTank(drivetrain, driverGamepad.leftYAxis, driverGamepad.rightYAxis,
        driverGamepad.rightAnalogTrigger));
  }

  public Command getAutonomousCommand() {
    return this.autonomousCommand;
  }

}
