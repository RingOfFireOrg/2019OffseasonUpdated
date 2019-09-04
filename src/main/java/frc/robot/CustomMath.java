package frc.robot;

public class CustomMath{

    static double limitRange(double input, double min, double max) {
        if (input < min) {
            input = min;
        } else if (input > max) {
            input = max;
        }
        return input;
    }
    
    static double returnGreater(double a, double b) {
        if (a > b) return a;
        else return b;
    }
}