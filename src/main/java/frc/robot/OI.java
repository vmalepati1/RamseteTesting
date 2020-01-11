package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class OI {
    public static final int SHIFT_UP_PORT = 3;
    public static final int SHIFT_DOWN_PORT = 2;

    public static final int LEFT_JOYSTICK_PORT = 0;
    public static final int RIGHT_JOYSTICK_PORT = 1;
    public static final Joystick leftJoystick = new Joystick(LEFT_JOYSTICK_PORT);
    public static final Joystick rightJoystick = new Joystick(RIGHT_JOYSTICK_PORT);

    public static final JoystickButton shiftUp = new JoystickButton(leftJoystick, SHIFT_UP_PORT);
    public static final JoystickButton shiftDown = new JoystickButton(leftJoystick, SHIFT_DOWN_PORT);

}
