/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.cameraserver.CameraServer;

public class Swapcameras extends CommandBase {

  // Modified from the multi-camera tutorial on WPILib:
  // https://docs.wpilib.org/en/stable/docs/software/vision-processing/roborio/using-multiple-cameras.html
  UsbCamera camera;
  VideoSink cameraServer;
  
  public Swapcameras(UsbCamera camera) {
    this.camera = camera;
    this.cameraServer = CameraServer.getServer();
    // configure the camera resolution so we get higher framerates
    this.camera.setVideoMode(PixelFormat.kMJPEG, 160, 120, 25);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (this.cameraServer.getSource() != this.camera) {
      this.cameraServer.setSource(this.camera);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
