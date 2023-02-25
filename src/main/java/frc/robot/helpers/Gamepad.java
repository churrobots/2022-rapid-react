package frc.robot.helpers;

import edu.wpi.first.wpilibj.Joystick;

public class Gamepad {

  public final Axis leftYAxis;
  public final Axis leftXAxis;
  public final Axis rightYAxis;
  public final Axis rightXAxis;
  public final Axis leftAnalogTrigger;
  public final Axis rightAnalogTrigger;
  // TODO: add other axes and buttons

  public Gamepad(int driverStationPort) {

    Joystick gamepad = new Joystick(driverStationPort);

    // TODO: allow different gamepad types
    // if (gamepadType == "LogitechF310") {

    leftXAxis = new Axis(gamepad, 0);
    leftYAxis = new Axis(gamepad, 1);
    rightXAxis = new Axis(gamepad, 4);
    rightYAxis = new Axis(gamepad, 5);

    leftAnalogTrigger = new Axis(gamepad, 2);
    rightAnalogTrigger = new Axis(gamepad, 3);
    // }

    // throw new InvalidGamepadTypeException(String.format("'%s' is not a recognized
    // gamepad type", gamepadType));

  }

  // We found this suggestion from the forums:
  // https://www.chiefdelphi.com/t/can-you-bind-a-command-to-only-activate-when-2-buttons-are-held/347368/4
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