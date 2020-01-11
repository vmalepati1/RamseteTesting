package frc.robot.command;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.OI.*;
import static frc.robot.Robot.drivetrain;

public class Drive extends CommandBase {

    public Drive() {
        addRequirements(drivetrain);
    }


    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

        if(Math.abs(leftJoystick.getY()) > 0.05 || Math.abs(rightJoystick.getY()) > 0.05) {
            drivetrain.setSpeeds(-leftJoystick.getY(), -rightJoystick.getY());
        }

        if (shiftUp.get()) {
            drivetrain.shiftUp();
        } else if (shiftDown.get()) {
            drivetrain.shiftDown();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
