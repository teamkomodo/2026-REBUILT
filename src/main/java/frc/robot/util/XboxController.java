package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class XboxController {
    CommandXboxController xboxController;

    public Trigger rt;
    public Trigger lt;
    public Trigger a;
    public Trigger b;
    public Trigger x;
    public Trigger y;
    public Trigger rb;
    public Trigger lb;
    public Trigger povDown;
    public Trigger povUp;
    public Trigger povLeft;
    public Trigger povRight;
    public Trigger rightStick;
    public Trigger leftStick;

    public XboxController(int port) {
        xboxController = new CommandXboxController(port);

        rt = xboxController.rightTrigger();
        lt = xboxController.leftTrigger();
        a = xboxController.a();
        b = xboxController.b();
        x = xboxController.x();
        y = xboxController.y();
        rb = xboxController.rightBumper();
        lb = xboxController.leftBumper();
        povDown = xboxController.povDown();
        povUp = xboxController.povUp();
        povLeft = xboxController.povLeft();
        povRight = xboxController.povRight();
        rightStick = xboxController.rightStick();
        leftStick = xboxController.leftStick();
    }
    
    public double getRightJoystickX() {
        return xboxController.getRightX();
    }

    public double getRightJoystickY() {
        return xboxController.getRightY();
    }

    public double getLeftJoystickX() {
        return xboxController.getLeftX();
    }

    public double getLeftJoystickY() {
        return xboxController.getLeftY();
    }

    public void rumbleSmooth(double intensity) {
        xboxController.setRumble(RumbleType.kRightRumble, getIntensity(intensity));
    }

    public void rumbleRough(double intensity) {
        xboxController.setRumble(RumbleType.kLeftRumble, getIntensity(intensity));
    }

    public void rumbleBoth(double intensity) {
        xboxController.setRumble(RumbleType.kLeftRumble, getIntensity(intensity));
        xboxController.setRumble(RumbleType.kRightRumble, getIntensity(intensity));
    }

    public void stopRoughRumble() {
        xboxController.setRumble(RumbleType.kLeftRumble, 0);
    }

    public void stopSmoothRumble() {
        xboxController.setRumble(RumbleType.kRightRumble, 0);
    }

    public double getIntensity(double intensity) {
        double absIntensity = Math.abs(intensity);
        if (absIntensity > 1.0) {
            absIntensity = 1.0;
        }
        return absIntensity;
    }
}
