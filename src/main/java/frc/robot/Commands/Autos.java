// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.MECHANISM;
import frc.robot.Constants.PATHING;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Mechanism;
import frc.robot.util.imaging.LimelightHelpers;

/** Container for auto command factories. */
public final class Autos {

  private static final Timer timer = new Timer();

  /**
   * A simple auto routine that drives backward a specified distance, and then
   * stops.
   */
  public static Command driveBack(Drivetrain drive) {
    return drive.positionDriveCommand(-100, -100).beforeStarting(drive::resetEncoders);
  }

  /**
   * A complex auto routine that drives backward, then balances
   */
  public static Command driveBackAndBalance(Drivetrain drive) {
    return Commands.sequence(
        // Drive onto the charging station
        drive.positionDriveCommand(-60, -60).beforeStarting(drive::resetEncoders),

        // Balance the robot
        drive.balance());
  }

  /**
   * A complex auto routine that places a game piece, picks up another one then
   * places it.
   */
  public static Command scoreTwoPieces(Drivetrain drive, Mechanism mech) {
    return Commands.sequence(

        // Raise arm to reach target
        mech.setArmPreset(MECHANISM.MID).until(() -> timer.advanceIfElapsed(2)),

        drive.positionDriveCommand(16, 16).beforeStarting(drive::resetEncoders).until(() -> timer.advanceIfElapsed(3)),

        // TODO: add drop game piece command

        drive.positionDriveCommand(-40, -40)
            .alongWith(mech.setArmPreset(MECHANISM.TRANSPORT)).until(() -> timer.advanceIfElapsed(3.5)),

        drive.positionDriveCommand(-72, -8).until(() -> timer.advanceIfElapsed(3.5)),

        mech.setArmPreset(MECHANISM.GROUND).until(() -> timer.advanceIfElapsed(2)),

        drive.positionDriveCommand(-52, 12).until(() -> timer.advanceIfElapsed(3)),

        // TODO: add ground pickup command

        mech.setArmPreset(MECHANISM.TRANSPORT).until(() -> timer.advanceIfElapsed(3)),

        drive.positionDriveCommand(-72, -8).until(() -> timer.advanceIfElapsed(3)),

        drive.positionDriveCommand(-40, -40).until(() -> timer.advanceIfElapsed(3)),

        drive.positionDriveCommand(12, 12).alongWith(mech.setArmPreset(MECHANISM.MID))
            .until(() -> timer.advanceIfElapsed(5)),

        // TODO: add score game piece command

        drive.positionDriveCommand(0, 0).alongWith(mech.setArmPreset(MECHANISM.DEFAULT))
            .until(() -> timer.advanceIfElapsed(5)))
        .beforeStarting(timer::restart).andThen(timer::stop);
  }

  /**
   * A highly sophisticated auto routine that places a cone on the middle peg,
   * drives back, turns around, grabs another cone, and then drives
   * to the charging station and balances.
   */
  public static Command scoreAndBalance(Drivetrain drive) {
    return Commands.sequence(
        // Drive onto the charging station
        drive.positionDriveCommand(0, 0),

        // Balance the robot
        drive.balance()).beforeStarting(drive::resetEncoders);
  }

  /**
   * Assuming this method is part of a drivetrain subsystem that provides the
   * necessary methods
   */
  public static Command followTrajectoryCommand(Drivetrain drive, PathPlannerTrajectory traj, boolean isFirstPath) {
    return Commands.sequence(
        new InstantCommand(() -> {
          // Reset odometry for the first path you run during auto
          if (isFirstPath) {
            drive.resetOdometry(traj.getInitialPose());
          }
        }),
        new PPRamseteCommand(
            traj,
            drive::getPose, // Pose supplier
            new RamseteController(
                PATHING.RAMSETE_B,
                PATHING.RAMSETE_ZETA),
            new SimpleMotorFeedforward(
                PATHING.kS,
                PATHING.kV,
                PATHING.kA),
            PATHING.DRIVE_KINEMATICS, // DifferentialDriveKinematics
            drive::getWheelSpeeds, // DifferentialDriveWheelSpeeds supplier
            new PIDController(PATHING.kP, 0, 0), // Left controller. Tune these values for your robot. Leaving them 0
                                                 // will only
            // use feedforwards.
            new PIDController(PATHING.kP, 0, 0), // Right controller (usually the same values as left controller)
            drive::setVoltageOutput, // Voltage biconsumer
            true, // Should the path be automatically mirrored depending on alliance color.
                  // Optional, defaults to true
            drive // Requires this drive subsystem
        ));
  }

  /**
   * Uses botpose to go to an AprilTag
   * 
   * @return Autonomous command
   */
  public static Command goToAprilTag(Drivetrain drive) {
    Optional<Pose2d> aprilTagCoords = Optional.ofNullable(LimelightHelpers.getTargetPose3d_RobotSpace("limelight").toPose2d());
    
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      // Start at the origin
      new Pose2d(0, 0, new Rotation2d(0)),
      // Pass through no waypoints
      List.of(),
      // Pass Apriltag coordinates
      aprilTagCoords.orElse(new Pose2d(0, 0, new Rotation2d(0))),
      // Pass config
      PATHING.PATH_CONFIG
    ); 

    return drive.ramseteTrajectory(trajectory);
  } 

  public static Command loadPathAutos(Drivetrain drive, Mechanism mech) {
    Map<String, Command> eventMap = new HashMap<>();
    // eventMap.put("event1", mech.setArmPreset(MECHANISM.MID));
    eventMap.put("event1", mech.setArmPreset(MECHANISM.MID).withTimeout(4));
    RamseteAutoBuilder autoBuilder = new RamseteAutoBuilder(
      drive::getPose,
      drive::resetOdometry,
      new RamseteController(PATHING.RAMSETE_B, PATHING.RAMSETE_ZETA),
      PATHING.DRIVE_KINEMATICS,
      new SimpleMotorFeedforward(
        PATHING.kS,
        PATHING.kV,
        PATHING.kA),
      drive::getWheelSpeeds,
      new PIDConstants(PATHING.kP, 0, 0),
      drive::setVoltageOutput, eventMap, drive
    );

    List<PathPlannerTrajectory> paths = PathPlanner.loadPathGroup("TestPathC", PATHING.TRAJECTORY_MAX_SPEED, PATHING.TRAJECTORY_MAX_ACCEL);


    // Command autoTest = new SequentialCommandGroup(
    //   new FollowPathWithEvents(
    //     followTrajectoryCommand(drive, paths.get(0), true),
    //     paths.get(0).getMarkers(), 
    //     eventMap),
    //   new WaitCommand(5),
    //   new FollowPathWithEvents(
    //     followTrajectoryCommand(drive, paths.get(1), false),
    //     paths.get(1).getMarkers(), 
    //     eventMap)
    // );
    // return autoTest;

    // Command autoTest = new SequentialCommandGroup(
    //   new FollowPathWithEvents(
    //     followTrajectoryCommand(drive, paths.get(0), true),
    //     paths.get(0).getMarkers(), 
    //     eventMap),
    //   mech.setArmPreset(MECHANISM.MID),
    //   new FollowPathWithEvents(
    //     followTrajectoryCommand(drive, paths.get(1), false),
    //     paths.get(1).getMarkers(), 
    //     eventMap)
    // );
    // return autoTest;


    PathPlannerTrajectory trajectory = PathPlanner.loadPath("TestPathC", new PathConstraints(3, 1));
    return autoBuilder.fullAuto(trajectory);
  }



  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}