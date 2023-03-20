package frc.robot.util.control;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmPresets {

    // PID coefficients
    public double wristHeight;
    public double wristOffset;

    /**
     * PID Values for Basic Balancing Algorithm
     * 
     * @param wristHeight
     * @param wristOffset
     */
    public ArmPresets(double height, double offset) {
        this.wristHeight = height;
        this.wristOffset = offset;
    }

    /**
     * Push preset properties to the Smart Dashboard
     * 
     * @param name
     */
    public void pushToDashboard(String name) {
        SmartDashboard.putNumber(name + " Wrist Height", wristHeight);
        SmartDashboard.putNumber(name + " Wrist Offset", wristOffset);
    }

    /**
     * Retrieve preset properties from the Smart Dashboard
     * 
     * @param name
     */
    public void getFromDashboard(String name) {
        this.wristHeight = SmartDashboard.getNumber(name + " Wrist Height", 0);
        this.wristOffset = SmartDashboard.getNumber(name + " Wrist Offset", 0);
    }

    /**
     * @return String representation of preset properties
     */
    @Override
    public String toString() {
        return String.format("Wrist Height: %f, Wrist Offset: %f\n", wristHeight, wristOffset);
    }

}
