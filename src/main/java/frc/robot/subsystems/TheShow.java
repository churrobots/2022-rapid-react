// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.TimerTask;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Drawings;

public class TheShow extends SubsystemBase {

  // NOTE: you can only allocate ONE strip of LEDs (in series). This is a
  // limitation
  // of how WPILib implements the addressable LED strips.
  // https://www.chiefdelphi.com/t/can-not-allocate-a-second-addressableled-pwm-port/376859

  // Our lights are set up as two 8x32 grids in series.
  AddressableLED leds = new AddressableLED(Constants.lightsPWM);
  final static int ROWS = 8;
  final static int COLS = 32;
  final static int STRIPS = 2;
  final static int PIXELS = (ROWS * COLS) * STRIPS;
  AddressableLEDBuffer emptyPixels = new AddressableLEDBuffer(PIXELS);

  Timer t = new Timer();

  static AddressableLEDBuffer wakaWakaPixels = new AddressableLEDBuffer(PIXELS);

  private static void populateWakaWaka(int red, int green, int blue, int[][] wakawaka) {
    int i = 0;
    for (int c = 0; c < wakawaka.length; c++) {
      if (c % 2 != 0) {
        for (int r = 0; r < wakawaka[c].length; r++) {
          if (wakawaka[c][r] == 1) {
            wakaWakaPixels.setRGB(i, red, green, blue);
          }
          i++;
        }
      } else {
        for (int r = wakawaka[c].length - 1; r >= 0; r--) {
          if (wakawaka[c][r] == 1) {
            wakaWakaPixels.setRGB(i, red, green, blue);
          }
          i++;
        }
      }
    }
  }

  // private static void populateWakaWakaFull(int red, int green, int blue) {
  //   // for (int i = 2; i < 6; i++) {
  //   // wakaWakaPixels.setRGB(i, red, green, blue);
  //   // }
  //   // for (int i = 9; i < 15; i++) {
  //   // wakaWakaPixels.setRGB(i, red, green, blue);
  //   // }
  //   // for (int i = 16; i < 48; i++) {
  //   // wakaWakaPixels.setRGB(i, red, green, blue);
  //   // }
  //   // for (int i = 49; i < 55; i++) {
  //   // wakaWakaPixels.setRGB(i, red, green, blue);
  //   // }
  //   // for (int i = 58; i < 62; i++) {
  //   // wakaWakaPixels.setRGB(i, red, green, blue);
  //   // }
  //   int i = 0;
  //   for (int c = 0; c < Drawings.mouth.length; c++) {
  //     if (c % 2 != 0) {
  //       for (int r = 0; r < Drawings.mouth[c].length; r++) {
  //         if (Drawings.mouth[c][r] == 1) {
  //           wakaWakaPixels.setRGB(i, red, green, blue);
  //         }
  //         i++;
  //       }
  //     } else {
  //       for (int r = Drawings.mouth[c].length - 1; r >= 0; r--) {
  //         if (Drawings.mouth[c][r] == 1) {
  //           wakaWakaPixels.setRGB(i, red, green, blue);
  //         }
  //         i++;
  //       }
  //     }
  //   }
  // }

  public TheShow() {
    leds.setLength(PIXELS);
    leds.start();
    t.start();
    // populateWakaWaka(100, 40, 0);
  }

  public int getPixelCount() {
    return PIXELS;
  }

  public void clearPixels() {
    leds.setData(emptyPixels);
    populateWakaWaka(0, 0, 0, Drawings.empty);
  }

  public void fill(int red, int green, int blue) {
    AddressableLEDBuffer pixels = new AddressableLEDBuffer(PIXELS);
    for (int i = 0; i < PIXELS; i++) {
      pixels.setRGB(i, red, green, blue);
    }
    leds.setData(pixels);
  }

  public void wakaWaka(int[][] wakawaka) {
    populateWakaWaka(100, 45, 0, wakawaka);
    leds.setData(wakaWakaPixels);
  }

  public void wakaWakaButBetter() {
    if (t.get() <= 0.5) {
      //this.wakaWaka(Drawings.fullMouth);
      this.wakaWaka(Drawings.circle);
    } else if (t.get() <= 1) {
      this.wakaWaka(Drawings.halfMouth);
    } else if (t.get() <= 1.5) {
      //this.wakaWaka(Drawings.circle);
      this.wakaWaka(Drawings.fullMouth);
    } else {
      t.reset();
      this.clearPixels();

    }

  }

  public void setPixels(AddressableLEDBuffer buffer) {
    boolean isError = buffer.getLength() != PIXELS;
    if (isError) {
      fill(255, 0, 0);
      return;
    }
  }

  @Override
  public void periodic() {
    if (RobotState.isDisabled()) {
      runDefaultLights();
    }
  }

  private void runDefaultLights() {
    // this.wakaWaka(100, 45, 0);
    this.wakaWakaButBetter();
  }
}
