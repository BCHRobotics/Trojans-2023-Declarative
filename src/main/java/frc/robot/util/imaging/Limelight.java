package frc.robot.util.imaging;

import java.awt.geom.Line2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.MISC;
import frc.robot.Constants.PATHING;

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
     * Get bot pose (position and rotation) in 3D field space
     * 
     * @return
     */
    public double[] getBotPose() {
        return this.networkTable.getEntry("botpose").getDoubleArray(new double[6]);
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

    /**
     * Creates a trajectory from the robot's current position to a pose (position and rotation) on the field
     * 
     * @param target in metres respective to the center of the field and degrees clockwise from positive x
     * @param limits defined in metres
     * @return smooth trajectory to target
     */
    public Trajectory getTrajectoryToTarget(Pose2d target, Rectangle2D[] limits) {
        //Initialize arrays for sorting limits
        Rectangle2D[] orderedLimitsArray = new Rectangle2D[limits.length];
        double[] distanceToLimitsArray = new double[limits.length];
        int[] sortedIndices = new int[limits.length];
        for(int i = 0; i < limits.length; i++) {
            distanceToLimitsArray[i] = Math.hypot(limits[i].getCenterX() - this.getBotPose()[0], limits[i].getCenterY() - this.getBotPose()[1]);
            sortedIndices[i] = i;
        }
        
        //Sort limits by distance from bot
        double tempDistance = 0;
        int tempIndex = 0;
        for(int i = 0; i < distanceToLimitsArray.length; i++) {
            for(int j = i + 1; j < distanceToLimitsArray.length; j++) {
                if(distanceToLimitsArray[i] > distanceToLimitsArray[j]) {
                    tempDistance = distanceToLimitsArray[i];
                    distanceToLimitsArray[i] = distanceToLimitsArray[j];
                    distanceToLimitsArray[j] = tempDistance;

                    tempIndex = sortedIndices[i];
                    sortedIndices[i] = sortedIndices[j];
                    sortedIndices[j] = tempIndex;
                }
            }
        }

        //Apply sorting
        for(int i = 0; i < sortedIndices.length; i++) {
            orderedLimitsArray[i] = limits[sortedIndices[i]];
        }

        ArrayList<Translation2d> intermediaryPoints = new ArrayList<Translation2d>();
        Line2D lineToTarget = new Line2D.Double(this.getBotPose()[0], this.getBotPose()[1], target.getX(), target.getY());
        
        //Find intermdiary points
        for(Rectangle2D limit : orderedLimitsArray) {
            //Continue adding points until the line between the most recently added point and the target no longer intersects the current limit
            //Shouldn't loop more than twice
            while(limit.intersectsLine(lineToTarget)) {
                Translation2d[] corners = 
                    {new Translation2d(limit.getMinX(), limit.getMinY()), new Translation2d(limit.getMaxX(), limit.getMinY()),
                     new Translation2d(limit.getMinX(), limit.getMaxY()), new Translation2d(limit.getMaxX(), limit.getMaxY())};

                //Find which corner of the rectangle is closest to the beginning of the line between the most recently added point and the target
                double shortestDistance = 1e100;
                int closestCorner = -1;
                for(int i = 0; i < corners.length; i++) {
                    double distanceToCorner = Math.hypot(corners[i].getX() - lineToTarget.getX1(), corners[i].getY() - lineToTarget.getY1());
                    if(distanceToCorner < shortestDistance) {
                        closestCorner = i;
                        shortestDistance = distanceToCorner;
                    }
                }

                intermediaryPoints.add(corners[closestCorner]);
                lineToTarget = new Line2D.Double(this.getBotPose()[0], this.getBotPose()[1], corners[closestCorner].getX(), corners[closestCorner].getY());
            }
        }

        //Generate trajectory
        Pose2d botPose = new Pose2d(this.getBotPose()[0], this.getBotPose()[1], new Rotation2d(this.getBotPose()[3]));
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(PATHING.TRAJECTORY_MAX_SPEED, PATHING.TRAJECTORY_MAX_SPEED);
        return TrajectoryGenerator.generateTrajectory(botPose, intermediaryPoints, target, trajectoryConfig);
    }

}