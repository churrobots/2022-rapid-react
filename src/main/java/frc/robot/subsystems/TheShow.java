// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TheShow extends SubsystemBase {

  // NOTE: you can only allocate ONE strip of LEDs (in series). This is a limitation
  // of how WPILib implements the addressable LED strips.
  // https://www.chiefdelphi.com/t/can-not-allocate-a-second-addressableled-pwm-port/376859

  // Our lights are set up as two 8x32 grids in series.
  AddressableLED leds = new AddressableLED(Constants.lightsPWM);
  final int ROWS = 8;
  final int COLS = 32;
  final int STRIPS = 2;
  final int PIXELS = (ROWS * COLS) * STRIPS;
  AddressableLEDBuffer priorBuffer = new AddressableLEDBuffer(PIXELS);

  public TheShow() {
    leds.setLength(PIXELS);
    leds.start();
  }

  public int getPixelCount() {
    return PIXELS;
  }

  public void clearPixels() {
    fill(0, 0, 0);
  }

  public void fill(int red, int green, int blue) {
    AddressableLEDBuffer pixels = new AddressableLEDBuffer(PIXELS);
    for (int i = 0; i < PIXELS; i++) {
      pixels.setRGB(i, red, green, blue);
    }
    leds.setData(pixels);
  }

  public void setPixels(AddressableLEDBuffer buffer) {
    boolean isError = buffer.getLength() != PIXELS;
    if (isError) {
      fill(255, 0, 0);
      return;
    }
    if (!priorBuffer.equals(buffer)) {
      leds.setData(buffer);
    }
  }

  @Override
  public void periodic() {
    if (RobotState.isDisabled()) {
      runDefaultLights();
    }
  }

  private void runDefaultLights() {
    this.fill(255, 0, 0);
  }
}
