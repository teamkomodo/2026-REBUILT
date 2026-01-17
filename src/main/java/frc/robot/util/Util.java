package frc.robot.util;


public class Util {

    public static double translationCurve(double input) {
        return Math.pow(Math.abs(input), 1.5) * Math.signum(input);
    }

    public static double steerCurve(double input) {
        return Math.pow(input, 3);
    }

}