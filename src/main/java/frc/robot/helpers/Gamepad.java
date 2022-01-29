package frc.robot.helpers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

public class Gamepad {

  public final Button aButton;
  public final Button bButton;
  public final Button xButton;
  public final Button yButton;

  public final Button povUp;
  public final Button povDown;
  public final Button povLeft;
  public final Button povRight;

  public final Button backButton;
  public final Button startButton;

  public final Button leftBumper;
  public final Button rightBumper;

  public final Axis leftYAxis;
  public final Axis leftXAxis;
  public final Axis rightYAxis;
  public final Axis rightXAxis;
  public final Axis leftAnalogTrigger;
  public final Axis rightAnalogTrigger;
  // TODO: add other axes and buttons

  public Gamepad(int driverStationPort)  {

    Joystick gamepad = new Joystick(driverStationPort);

    // TODO: allow different gamepad types
    // if (gamepadType == "LogitechF310") {

      aButton = new JoystickButton(gamepad, 1);
      bButton = new JoystickButton(gamepad, 2);
      xButton = new JoystickButton(gamepad, 3);
      yButton = new JoystickButton(gamepad, 4);

      backButton = new JoystickButton(gamepad, 7);
      startButton = new JoystickButton(gamepad, 8);

      povUp = new POVButton(gamepad, 0);
      povDown = new POVButton(gamepad, 180);
      povLeft = new POVButton(gamepad, 270);
      povRight = new POVButton(gamepad, 90);

      leftBumper = new JoystickButton(gamepad, 5);
      rightBumper = new JoystickButton(gamepad, 6);

      leftXAxis = new Axis(gamepad, 0);
      leftYAxis = new Axis(gamepad, 1);
      rightXAxis = new Axis(gamepad, 4);
      rightYAxis = new Axis(gamepad, 5);

      leftAnalogTrigger = new Axis(gamepad, 2);
      rightAnalogTrigger = new Axis(gamepad, 3);
    // }

    // throw new InvalidGamepadTypeException(String.format("'%s' is not a recognized gamepad type", gamepadType));

  }

  public Button getDualButton(Button button1, Button button2) {
    Button dualButton = new DualButton(button1, button2);
    return dualButton;
  }

  // We found this suggestion from the forums:
  // https://www.chiefdelphi.com/t/can-you-bind-a-command-to-only-activate-when-2-buttons-are-held/347368/4
  protected static class DualButton extends Button {

    Button a;
    Button b;

    public DualButton(Button a, Button b) {
      this.a = a;
      this.b = b;
    }

    @Override
    public boolean get() {
      return a.get() && b.get();
    }

  }

  public static class Axis {

    private Joystick _gamepad;
    private int _axis;

    Axis(Joystick gamepad, int axis) {
      _gamepad = gamepad;
      _axis = axis;
    }

    public double get() {
      return _gamepad.getRawAxis(_axis);
    }

  }

  public static class InvalidGamepadTypeException extends RuntimeException {

    public InvalidGamepadTypeException(String message) {
      super(message);
    }

    private static final long serialVersionUID = -6464474179450099666L;

  }

}