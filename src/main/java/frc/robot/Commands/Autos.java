// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.MECHANISM;
import frc.robot.Constants.PATHING;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Mechanism;

/** Container for auto command factories. */
public final class Autos {

  private static final Timer timer = new Timer();

  /**
   * A simple auto routine that drives backward a specified distance, and then
   * stops.
   */
  public static Command driveBack(Drivetrain drive, Mechanism mech) {
    return Commands.sequence(
        // mech.setArmPreset(MECHANISM.STOWED).until(() -> timer.advanceIfElapsed(1)),
        mech.setArmPreset(MECHANISM.STOWED).withTimeout(1),

        mech.launchGamePiece()
            // .until(() -> timer.advanceIfElapsed(1)),
            .withTimeout(1),

        // Drive onto the charging station
        drive.positionDriveCommand(-160, -160)
            .beforeStarting(Commands.runOnce(drive::resetEncoders)));
  }

  /**
   * A simple auto routine that turns 180 degrees, and then stops.
   */
  public static Command turn(Drivetrain drive) {
    return drive.positionDriveCommand(31, -31).beforeStarting(Commands.runOnce(drive::resetEncoders));
  }

  /**
   * A complex auto routine that drives backward, then balances
   */
  public static Command balance(Drivetrain drive) {
    return Commands.sequence(
        // Drive onto the charging station
        drive.positionDriveCommand(94, 94)
            .until(() -> timer.advanceIfElapsed(6)),

        // Balance the robot
        drive.balance())
        .beforeStarting(Commands.runOnce(timer::restart))
        .beforeStarting(Commands.runOnce(drive::resetEncoders))
        .andThen(Commands.runOnce(timer::stop));
  }

  /**
   * A complex auto routine that places a game piece, picks up another one then
   * places it.
   */
  public static Command scoreTwoPieces(Drivetrain drive, Mechanism mech) {
    return Commands.sequence(

        // Raise arm to reach target
        mech.setArmPreset(MECHANISM.MID).until(() -> timer.advanceIfElapsed(1)),

        drive.positionDriveCommand(16, 16).until(() -> timer.advanceIfElapsed(2)),

        drive.positionDriveCommand(-40, -40)
            .alongWith(mech.releaseGamePiece().andThen(mech.setArmPreset(MECHANISM.HOME)))
            .until(() -> timer.advanceIfElapsed(6)),

        drive.positionDriveCommand(-71, -9).alongWith(mech.setArmPreset(MECHANISM.LOW))
            .until(() -> timer.advanceIfElapsed(6)),

        drive.positionDriveCommand(-51, 11).until(() -> timer.advanceIfElapsed(4)),

        mech.grabCube(),

        drive.positionDriveCommand(-71, -9).alongWith(mech.setArmPreset(MECHANISM.HOME))
            .until(() -> timer.advanceIfElapsed(4)),

        drive.positionDriveCommand(-40, -40).until(() -> timer.advanceIfElapsed(4)),

        drive.positionDriveCommand(20, 20).alongWith(mech.setArmPreset(MECHANISM.LOW))
            .until(() -> timer.advanceIfElapsed(4)),

        mech.releaseGamePiece(),

        drive.positionDriveCommand(0, 0).alongWith(mech.setArmPreset(MECHANISM.HOME))
            .until(() -> timer.advanceIfElapsed(4)))
        .beforeStarting(timer::restart);
  }

  /**
   * A highly sophisticated auto routine that places a gamepiece in the hybrid
   * zone,
   * drives back, and then drives to the charging station and balances.
   */
  public static Command scoreAndBalance(Drivetrain drive, Mechanism mech) {
    return Commands.sequence(
        mech.setArmPreset(MECHANISM.STOWED).until(() -> timer.advanceIfElapsed(1)),
        mech.releaseGamePiece().until(() -> timer.advanceIfElapsed(1)),

        // Drive onto the charging station
        drive.positionDriveCommand(-95, -95)
            .alongWith(mech.setArmPreset(MECHANISM.HOME))
            .until(() -> timer.advanceIfElapsed(6)),

        // Balance the robot
        drive.balance())
        .beforeStarting(Commands.runOnce(timer::restart))
        .beforeStarting(Commands.runOnce(drive::resetEncoders))
        .andThen(Commands.runOnce(timer::stop));
  }

  /**
   * A highly sophisticated auto routine that places a gamepiece in the hybrid
   * zone,
   * drives back, and then drives to the charging station and balances.
   */
  public static Command mobilityAndBalance(Drivetrain drive, Mechanism mech) {
    return Commands.sequence(
        mech.setArmPreset(MECHANISM.STOWED).until(() -> timer.advanceIfElapsed(1)),
        mech.releaseGamePiece().until(() -> timer.advanceIfElapsed(1)),

        // Drive onto the charging station
        drive.positionDriveCommand(-170, -170)
            .alongWith(mech.setArmPreset(MECHANISM.HOME))
            .until(() -> timer.advanceIfElapsed(7)),

        // Drive to gyro heading
        drive.turnToGyro(0).until(() -> timer.advanceIfElapsed(2)),

        // Drive reset encoders
        Commands.runOnce(drive::resetEncoders),

        // Drive onto the charging station
        drive.positionDriveCommand(90, 90).until(() -> timer.advanceIfElapsed(2)),

        // Balance the robot
        drive.balance())
        .beforeStarting(Commands.runOnce(timer::restart))
        .beforeStarting(Commands.runOnce(drive::resetEncoders))
        .beforeStarting(Commands.runOnce(drive::resetGyro))
        .andThen(Commands.runOnce(timer::stop));
  }

  /**
   * A highly sophisticated auto routine that places a cone on the middle peg
   * autonomously, this is for driver use during teleop.
   */
  public static Command automatedScoringCommand(Drivetrain drive, Mechanism mech) {
    return Commands.sequence(
        drive.seekTarget(),

        mech.setArmPreset(MECHANISM.MID).alongWith(drive.goToTarget()),

        mech.releaseGamePiece());
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

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}