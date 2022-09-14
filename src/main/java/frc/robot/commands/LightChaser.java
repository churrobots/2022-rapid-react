package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TheShow;

public class LightChaser extends CommandBase {
  TheShow theShow;
  AddressableLEDBuffer led_Buffer;
  int litLight;

  public LightChaser(TheShow theShow) {
    this.theShow = theShow;
    this.led_Buffer = new AddressableLEDBuffer(theShow.getPixelCount());
    addRequirements(this.theShow);
  }

  @Override
  public void execute() {
    // Clear the stripe
    for (var i = 0; i < led_Buffer.getLength(); i++) {
      led_Buffer.setRGB(i, 0, 0, 0);
    }
    // Light up the lit light
    led_Buffer.setRGB(litLight, 0, 0, 255);
    litLight += 1;
    // Move the lit light to the next pixel.
    if (litLight >= led_Buffer.getLength() - 1) {
      litLight = 0;
    }
   
   
   theShow.setPixels(led_Buffer);
  }

  @Override
  public void end(boolean interrupted) {
    theShow.clearPixels();
  }


}
