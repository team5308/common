package frc.robot.org.team5165.common;

public class Utils {
    public static double normalize(double value, int cycle) {
        return value - cycle * ((int) value / cycle - (value < 0 ? 1 : 0));
    }
}
