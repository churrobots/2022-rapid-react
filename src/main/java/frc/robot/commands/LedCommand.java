package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class LedCommand extends CommandBase {
  AddressableLED led;
  AddressableLEDBuffer led_Buffer;
  int litLight;

  public LedCommand() {
    led = new AddressableLED(9);
    led_Buffer = new AddressableLEDBuffer(151);
    led.setLength(led_Buffer.getLength());

    // Set the data
    led.setData(led_Buffer);
    led.start();
  }

  @Override
  public void execute() {
    for (var i = 0; i < led_Buffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      led_Buffer.setRGB(i, 0, 0, 0);
    }
    led_Buffer.setRGB(litLight, 0, 0, 255);
    litLight += 1;
    if (litLight >= led_Buffer.getLength() - 1) {
      litLight = 0;
    }
   
   
   led.setData(led_Buffer);
  }

  @Override
  public void end(boolean interrupted) {
    for (var i = 0; i < led_Buffer.getLength(); i++) {
      led_Buffer.setRGB(i, 0, 0, 0);
    }
    led.setData(led_Buffer);
  }


}
