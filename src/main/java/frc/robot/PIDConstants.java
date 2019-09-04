package frc.robot;

public class PIDConstants {

    private double kP, kI, kD;

    public PIDConstants(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kD;
        this.kD = kD;
    }

    public double getKP() {
        return kP;
    }

    public double getKI() {
        return kI;
    }

    public double getKD() {
        return kD;
    }

}