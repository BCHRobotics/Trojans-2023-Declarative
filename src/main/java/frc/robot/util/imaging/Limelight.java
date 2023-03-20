package frc.robot.util.imaging;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.MISC;

public class Limelight {
    private static Limelight instance;

    private NetworkTable networkTable;
    private LimelightTargetType currentTarget;

    public static Limelight getInstance() {
        if (instance == null) {
            instance = new Limelight();
        }
        return instance;
    }

    public enum LimelightTargetType {
        CONE, CUBE, APRILTAG, NOTHING
    }

    public Limelight() {
        this.networkTable = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public void setDesiredTarget(LimelightTargetType target) {
        this.currentTarget = target;
    }

    public void setPipeline() {
        this.networkTable.getEntry("pipeline").setNumber(7);
    }

    public LimelightTargetType getDesiredTarget() {
        return this.currentTarget;
    }

    public LimelightTarget getTargetInfo() {

        LimelightTarget desiredTarget = new LimelightTarget(this.currentTarget);

        if (currentTarget == LimelightTargetType.CONE || currentTarget == LimelightTargetType.CUBE
                || currentTarget == LimelightTargetType.APRILTAG) {

            desiredTarget.setX(getTargetDistance());

            desiredTarget.setY(getTargetDistance());

            desiredTarget.setTargetFound(getTargetExists());

        } else {
            desiredTarget.setTargetFound(false);
        }

        return desiredTarget;
    }

    /**
     * Change the mode of the led 1-off, 2-blink, 3-on
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
        double a1 = MISC.LIMELIGHT_ANGLE; // Limelight mount angle
        double a2 = Math.abs(this.getTargetY()); // Limelight measured angle to target
        double aT = a1 + a2; // Total anlge in degrees
        double h1 = MISC.LIMELIGHT_HEIGHT; // Limelight lens Height in inches
        double h2 = MISC.APRILTAG_HEIGHT; // Known Height of Target in inches

        double distance = (h1 - h2) / Math.tan(Math.toRadians(aT));

        distance = Math.abs(Math.round(distance * 100.0) / 100.0);
        SmartDashboard.putNumber("Distance", distance);
        return distance; // in inches
    }

    /**
     * Checks if limelight reached target x angle
     * 
     * @return reached target
     */
    public boolean reachedTargetX() {
        return (MISC.WITHIN_TOLERANCE(getTargetX(), MISC.LIMELIGHT_TOLERANCE));
    }

}