/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.commands.DriveManually;
import frc.robot.commands.EjectLeft;
import frc.robot.commands.EjectRight;
import frc.robot.commands.MoveArmDown;
import frc.robot.commands.MoveArmUp;
import frc.robot.commands.Vacuum;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IntakeLeft;
import frc.robot.subsystems.IntakeRight;
import frc.robot.commands.AssistedClimb;
import frc.robot.commands.AutoDriveOffTarmac;
import frc.robot.commands.AutoDump;
import frc.robot.commands.Calibrating;
import frc.robot.helpers.Gamepad;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Connect to all the inputs (gamepads and shuffleboard).
  Gamepad driverGamepad = new Gamepad(Constants.driverGamepadPort); 
  Gamepad operatorGamepad = new Gamepad(Constants.operatorGamepadPort);

  // Connect to all the outputs.
  Drivetrain drivetrain = new Drivetrain();
  IntakeLeft polterLeftGust3000 = new IntakeLeft();
  IntakeRight polterRightGust3000 = new IntakeRight();
  Arm muscleArm = new Arm();

  // Create the autonomous chooser.
  SendableChooser<Command> autonomousChooser = new SendableChooser<Command>();

  public RobotContainer() {
  
    // Enable the camera.
    CameraServer.startAutomaticCapture();

    // Wire up commands to the controllers.
    drivetrain.setDefaultCommand(new DriveManually(drivetrain, driverGamepad.leftYAxis, driverGamepad.rightYAxis,
        driverGamepad.rightXAxis));
    driverGamepad.aButton.whileHeld(new AssistedClimb(muscleArm, drivetrain));
    operatorGamepad.getDualButton(operatorGamepad.startButton, operatorGamepad.backButton)
        .whileHeld(new Calibrating(muscleArm));
    operatorGamepad.yButton.whenHeld(new Vacuum(polterLeftGust3000, polterRightGust3000));
    operatorGamepad.leftBumper.whenHeld(new EjectLeft(polterLeftGust3000));
    operatorGamepad.rightBumper.whenHeld(new EjectRight(polterRightGust3000));
    operatorGamepad.povUp.whenPressed(new MoveArmUp(muscleArm));
    operatorGamepad.povDown.whenPressed(new MoveArmDown(muscleArm));

    // Set the options for autonomous.
    Command dump = new AutoDump(muscleArm, polterLeftGust3000, polterRightGust3000);
    Command drive = new AutoDriveOffTarmac(drivetrain);
    autonomousChooser.setDefaultOption("Dump", dump);
    autonomousChooser.addOption("Drive", drive);
    autonomousChooser.addOption("Dump and Drive", dump.andThen(drive));
    SmartDashboard.putData(autonomousChooser);

    // Show all the subsystems in the smartdashboard.
    SmartDashboard.putData(CommandScheduler.getInstance());
    SmartDashboard.putData((drivetrain));
    SmartDashboard.putData((polterLeftGust3000));
    SmartDashboard.putData((polterRightGust3000));
    SmartDashboard.putData((muscleArm));
  }

  public Command getAutonomousCommand() {
    // TODO: allow Shuffleboard to choose the auto mode
    return autonomousChooser.getSelected();
  }

}
