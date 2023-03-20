package frc.robot.util.control;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PIDConstants {

    // PID coefficients
    public double kP;
    public double kI;
    public double kD;
    public double kFF;
    public String name;

    /**
     * PID Values for Basic Balancing Algorithm
     * 
     * @param kP
     * @param kI
     * @param kD
     * @param kFF
     */
    public PIDConstants(double kP, double kI, double kD, double kFF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kFF = kFF;
    }

    public void pushToDashboard(String name) {
        this.name = name;
        SmartDashboard.putNumber(name + " P Gain", kP);
        SmartDashboard.putNumber(name + " I Gain", kI);
        SmartDashboard.putNumber(name + " D Gain", kD);
        SmartDashboard.putNumber(name + " Feed Forward", kFF);
    }

    public void getFromDashboard() {
        kP = SmartDashboard.getNumber(this.name + " P Gain", 0);
        kI = SmartDashboard.getNumber(this.name + " I Gain", 0);
        kD = SmartDashboard.getNumber(this.name + " D Gain", 0);
        kFF = SmartDashboard.getNumber(this.name + " Feed Forward", 0);
    }

    public boolean valuesChanged() {
        return (this.kP != SmartDashboard.getNumber(this.name + " P Gain", 0)
                || this.kI != SmartDashboard.getNumber(this.name + " I Gain", 0)
                || this.kD != SmartDashboard.getNumber(this.name + " D Gain", 0)
                || this.kFF != SmartDashboard.getNumber(this.name + " Feed Forward", 0));
    }

    @Override
    public String toString() {
        return String.format("kP: %f, kI: %f, kD: %f, kFF: %f\n", kP, kI, kD, kFF);
    }

}