package frc.robot.util;

public class MercMath {

    public static double squareInput(double input) {
        return input > 0.0 ? Math.pow(input, 2) : -Math.pow(input, 2);
    }

    public static double cubeInput(double input) {
        return Math.pow(input, 3);
    }
}
