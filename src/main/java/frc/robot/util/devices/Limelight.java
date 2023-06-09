package frc.robot.util.devices;

import frc.robot.Constants.MISC;
import frc.robot.Constants.VISION;
import frc.robot.Constants.VISION.TARGET_TYPE;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
    private static Limelight instance;

    private NetworkTable networkTable;
    private TARGET_TYPE currentTarget;
    private double targetHeight;

    public static Limelight getInstance() {
        if (instance == null) {
            instance = new Limelight();
        }
        return instance;
    }

    private Limelight() {
        this.networkTable = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public void setDesiredTarget(TARGET_TYPE target) {
        this.currentTarget = target;
        switch (target) {
            case CONE:
                this.targetHeight = VISION.CONE_TARGET_HEIGHT;
                this.setPipeline(VISION.CONE_PIPELINE);
                break;
            case CUBE:
                this.targetHeight = VISION.CUBE_TARGET_HEIGHT;
                this.setPipeline(VISION.CUBE_PIPELINE);
                break;
            case APRILTAG:
                this.targetHeight = VISION.APRILTAG_HEIGHT;
                this.setPipeline(VISION.APRILTAG_PIPELINE);
                break;
            case GAMEPIECE:
                this.targetHeight = 0;
                this.setPipeline(VISION.NEURAL_NETWORK_PIPELINE);
                break;
            default:
                this.targetHeight = 0;
                break;
        }
    }

    public TARGET_TYPE getDesiredTarget() {
        return this.currentTarget;
    }

    private void setPipeline(int pipeline) {
        this.networkTable.getEntry("pipeline").setNumber(pipeline);
    }

    /**
     * Change the mode of the led [0-default, 1-off, 2-blink, 3-on]
     * 
     * @param ledMode led mode
     */
    public void setLedMode(int ledMode) {
        this.networkTable.getEntry("ledMode").setNumber(ledMode);
    }

    public double getTargetX() {
        return this.networkTable.getEntry("tx").getDouble(0);
    }

    public double getTargetY() {
        return this.networkTable.getEntry("ty").getDouble(0);
    }

    public boolean getTargetExists() {
        return this.networkTable.getEntry("tv").getDouble(0) == 1;
    }

    public double getTargetArea() {
        return this.networkTable.getEntry("ta").getDouble(0);
    }

    /**
     * Get the distance to the target using Trigonometry
     * 
     * @return distance to target
     */
    public double getTargetDistance() {
        return Math.abs(Math.round(
                VISION.LIMELIGHT_HEIGHT - this.targetHeight / Math.tan(
                        Math.toRadians(VISION.LIMELIGHT_ANGLE + Math.abs(this.getTargetY()))) * 100.0)
                / 100.0); // distance from target in inches
    }

    /**
     * Checks if limelight reached target x angle
     * 
     * @return reached target
     */
    public boolean reachedTargetX() {
        return (MISC.WITHIN_TOLERANCE(getTargetX(), VISION.LIMELIGHT_TOLERANCE));
    }
}