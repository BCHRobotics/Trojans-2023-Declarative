// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.Constants.MECHANISM;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Mechanism;

/** Container for auto command factories. */
public final class Autos {

    /**
     * A simple auto routine that drives backward a specified distance, and then
     * stops.
     */
    public static Command driveBack(Drivetrain drive, Mechanism mech) {
        return Commands.sequence(
                mech.setArmPreset(MECHANISM.STOWED)
                        .withTimeout(1),

                mech.releaseGamePiece()
                        .withTimeout(1),

                // Drive onto the charging station
                drive.positionDriveCommand(-160, -160)
                        .alongWith(mech.setArmPreset(MECHANISM.HOME))
                        .withTimeout(6)
                        .beforeStarting(Commands.runOnce(drive::resetEncoders)));
    }

    /**
     * A complex auto routine that drives backward, then balances
     */
    public static Command balance(Drivetrain drive) {
        return Commands.sequence(
                // Drive onto the charging station
                drive.positionDriveCommand(94, 94)
                        .withTimeout(6),

                // Balance the robot
                drive.balance())
                .beforeStarting(Commands.runOnce(drive::resetEncoders));
    }

    /**
     * A complex auto routine that places a game piece, picks up another one then
     * places it.
     */
    public static Command scoreTwoPieces(Drivetrain drive, Mechanism mech) {
        return Commands.sequence(

                // Raise arm to reach target
                mech.setArmPreset(MECHANISM.MID)
                        .withTimeout(1),

                drive.positionDriveCommand(16, 16)
                        .withTimeout(2),

                drive.positionDriveCommand(-40, -40)
                        .alongWith(mech.releaseGamePiece()
                                .andThen(mech.setArmPreset(MECHANISM.HOME)))
                        .withTimeout(6),

                drive.positionDriveCommand(-71, -9)
                        .alongWith(mech.setArmPreset(MECHANISM.LOW))
                        .withTimeout(6),

                drive.positionDriveCommand(-51, 11)
                        .withTimeout(4),

                mech.grabCube(),

                drive.positionDriveCommand(-71, -9)
                        .alongWith(mech.setArmPreset(MECHANISM.HOME))
                        .withTimeout(4),

                drive.positionDriveCommand(-40, -40)
                        .withTimeout(4),

                drive.positionDriveCommand(20, 20)
                        .alongWith(mech.setArmPreset(MECHANISM.LOW))
                        .withTimeout(4),

                mech.releaseGamePiece(),

                drive.positionDriveCommand(0, 0)
                        .alongWith(mech.setArmPreset(MECHANISM.HOME))
                        .withTimeout(4));
    }

    /**
     * A highly sophisticated auto routine that places a gamepiece in the hybrid
     * zone,
     * drives back, and then drives to the charging station and balances.
     */
    public static Command scoreAndBalance(Drivetrain drive, Mechanism mech) {
        return Commands.sequence(
                mech.setArmPreset(MECHANISM.STOWED)
                        .withTimeout(1),

                mech.releaseGamePiece()
                        .withTimeout(1),

                // Drive onto the charging station
                drive.positionDriveCommand(-95, -95)
                        .alongWith(mech.setArmPreset(MECHANISM.HOME))
                        .withTimeout(6),

                // Balance the robot
                drive.balance())
                .beforeStarting(Commands.runOnce(drive::resetEncoders));
    }

    /**
     * A highly sophisticated auto routine that places a gamepiece in the hybrid
     * zone,
     * drives back, and then drives to the charging station and balances.
     */
    public static Command mobilityAndBalance(Drivetrain drive, Mechanism mech) {
        return Commands.sequence(
                mech.setArmPreset(MECHANISM.STOWED)
                        .withTimeout(1),

                mech.releaseGamePiece()
                        .withTimeout(1),

                // Drive onto the charging station
                drive.positionDriveCommand(-170, -170)
                        .alongWith(mech.setArmPreset(MECHANISM.HOME))
                        .withTimeout(7),

                // Drive to gyro heading
                drive.turnToGyro(0)
                        .withTimeout(2),

                // Drive reset encoders
                Commands.runOnce(drive::resetEncoders),

                // Drive onto the charging station
                drive.positionDriveCommand(90, 90)
                        .withTimeout(2),

                // Balance the robot
                drive.balance())
                .beforeStarting(Commands.runOnce(drive::resetEncoders))
                .beforeStarting(Commands.runOnce(drive::resetGyro));
    }

    /**
     * A highly sophisticated auto routine that places a cone on the middle peg
     * autonomously, this is for driver use during teleop.
     */
    public static Command automatedScoringCommand(Drivetrain drive, Mechanism mech) {
        return Commands.sequence(
                drive.seekTarget(),

                mech.setArmPreset(MECHANISM.MID)
                        .alongWith(drive.goToTarget()),

                mech.releaseGamePiece());
    }

    private Autos() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}