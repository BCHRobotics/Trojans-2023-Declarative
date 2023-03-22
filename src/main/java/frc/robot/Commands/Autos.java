// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.RamseteAutoBuilder;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.CHASSIS;
import frc.robot.Constants.MECHANISM;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Mechanism;

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
   * A basic auto that utilises RamseteCommand and PathPlannerLib to go in a circle.
   */
  public static Command pathPlannerAuto(Drivetrain drive, String pathName) {
    Map<String, Command> autoMap = new HashMap<>(); // No events for now;

    RamseteAutoBuilder autoBuilder = new RamseteAutoBuilder(drive::getPose, drive::resetPose, new RamseteController(), CHASSIS.DRIVE_KINEMATICS, drive::tankDriveVolts, autoMap, drive);
    PathPlannerTrajectory trajectory = PathPlanner.loadPath(pathName, CHASSIS.PATH_CONSTRAINTS);

    drive.resetPose(trajectory.getInitialPose());
    return autoBuilder.followPath(trajectory);
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}