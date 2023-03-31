package frc.robot.util.devices;

import frc.robot.Constants.MISC;
import frc.robot.Constants.VISION;
import frc.robot.Constants.VISION.TARGET_TYPE;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
            case REFLECTIVE_TAPE:
                this.targetHeight = VISION.REFLECTIVE_TAPE_HEIGHT;
                this.setPipeline(VISION.REFLECTIVE_PIPLINE);
                break;
            case APRILTAG:
                this.targetHeight = VISION.APRILTAG_HEIGHT;
                this.setPipeline(VISION.APRILTAG_PIPELINE);
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
     * Get bot pose (position and rotation) in 3D field space
     * 
     * @return
     */
    public double[] getBotPose() {
        return this.networkTable.getEntry("botpose").getDoubleArray(new double[6]);
    }

    /**
     * Get limelight april tag detected position
     * 
     * @return
     */
    public Pose2d getPose2d() {
        return new Pose2d(this.getBotPose()[0], this.getBotPose()[1], new Rotation2d(this.getBotPose()[3]));
    }

    /**
     * Get the distance to the target using Trigonometry
     * 
     * @return distance to target
     */
    public double getTargetDistance() {
        return Math.abs(
                Math.round(
                        ((VISION.LIMELIGHT_HEIGHT - this.targetHeight) /
                                Math.tan(VISION.LIMELIGHT_ANGLE - this.getTargetY())) * 10)
                        / 10); // distance from target in inches
    }

    /**
     * Checks if limelight reached target x angle
     * 
     * @return reached target
     */
    public boolean reachedTargetX() {
        return (MISC.WITHIN_TOLERANCE(getTargetX(), 0, VISION.LIMELIGHT_TOLERANCE));
    }
}