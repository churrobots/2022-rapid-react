/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.subsystems.Drivetrain;
import frc.robot.commands.DriveWithCurvature;
// import frc.robot.commands.ShowPacMan;
import frc.robot.helpers.Gamepad;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Connect to all the inputs (gamepads and shuffleboard).
  Gamepad m_driverController = new Gamepad(0);
  Gamepad m_operatorController = new Gamepad(1);

  // Connect to all the outputs.
  Drivetrain drivetrain = new Drivetrain();

  // Create the autonomous chooser.
  SendableChooser<Command> autonomousChooser = new SendableChooser<Command>();

  public RobotContainer() {

    // Enable the camera.
    CameraServer.startAutomaticCapture();

    // Default commands
    drivetrain.setDefaultCommand(new DriveWithCurvature(drivetrain, driverGamepad.leftYAxis,
        driverGamepad.rightXAxis));
    drivetrain.setDefaultCommand(new DriveWithCurvature(drivetrain,
        m_driverController));
  }

  public Command getAutonomousCommand() {
    return autonomousChooser.getSelected();
  }

  public Trajectory getTrajectory(String trajectoryJSON) {

    Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
    Trajectory trajectory = new Trajectory();
    try {
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException err) {
      DriverStation.reportError("broken!!!", err.getStackTrace());
    }
    return trajectory;
  }

}
