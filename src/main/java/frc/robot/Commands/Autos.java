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
        public static Command driveBackBlue(Drivetrain drive, Mechanism mech) {
                return Commands.sequence(
                                mech.setArmPreset(MECHANISM.STOWED)
                                                .withTimeout(1),

                                mech.releaseGamePiece()
                                                .withTimeout(1),

                                // Drive onto the charging station
                                drive.positionDriveCommand(-130, -130)
                                                .alongWith(mech.setArmPreset(MECHANISM.HOME))
                                                .withTimeout(5)
                                                .beforeStarting(Commands.runOnce(drive::resetEncoders)),

                                drive.positionDriveCommand(-32, 32)
                                                .alongWith(mech.setArmPreset(MECHANISM.LOW))
                                                .withTimeout(3)
                                                .beforeStarting(Commands.runOnce(drive::resetEncoders)));
        }

        /**
         * A simple auto routine that drives backward a specified distance, and then
         * stops.
         */
        public static Command driveBackRed(Drivetrain drive, Mechanism mech) {
                return Commands.sequence(
                                mech.setArmPreset(MECHANISM.STOWED)
                                                .withTimeout(1),

                                mech.releaseGamePiece()
                                                .withTimeout(1),

                                drive.positionDriveCommand(-130, -130)
                                                .alongWith(mech.setArmPreset(MECHANISM.HOME))
                                                .withTimeout(5)
                                                .beforeStarting(Commands.runOnce(drive::resetEncoders)),

                                drive.positionDriveCommand(32, -32)
                                                .alongWith(mech.setArmPreset(MECHANISM.LOW))
                                                .withTimeout(3)
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

                                mech.setArmPreset(MECHANISM.STOWED)
                                                .withTimeout(1),

                                mech.releaseGamePiece()
                                                .withTimeout(1),

                                // Drive onto the charging station
                                drive.positionDriveCommand(-120, -120)
                                                .beforeStarting(Commands.runOnce(drive::resetEncoders))
                                                .alongWith(mech.setArmPreset(MECHANISM.HOME))
                                                .withTimeout(6),

                                drive.positionDriveCommand(-33, 33)
                                                .beforeStarting(Commands.runOnce(drive::resetEncoders))
                                                .alongWith(mech.setArmPreset(MECHANISM.LOW))
                                                .withTimeout(4),

                                drive.positionDriveCommand(20, 20)
                                                .beforeStarting(Commands.runOnce(drive::resetEncoders))
                                                .alongWith(mech.grabCone())
                                                .withTimeout(4),

                                drive.positionDriveCommand(31, -31)
                                                .alongWith(mech.grabCone())
                                                .beforeStarting(Commands.runOnce(drive::resetEncoders))
                                                .withTimeout(3),

                                drive.positionDriveCommand(90, 90)
                                                .alongWith(mech.setArmPreset(MECHANISM.STOWED))
                                                .beforeStarting(Commands.runOnce(drive::resetEncoders))
                                                .withTimeout(4),

                                Autos.automatedScoringCommand(drive, mech));
        }

        /**
         * A simple auto routine that drives backward a specified distance, and then
         * stops.
         */
        public static Command scoreConeMid(Drivetrain drive, Mechanism mech) {
                return Commands.sequence(
                                Autos.automatedScoringCommand(drive, mech)
                                                .withTimeout(6),

                                drive.positionDriveCommand(-120, -120)
                                                .alongWith(mech.setArmPreset(MECHANISM.HOME))
                                                .withTimeout(5)
                                                .beforeStarting(Commands.runOnce(drive::resetEncoders)),

                                drive.positionDriveCommand(-32, 32)
                                                .alongWith(mech.setArmPreset(MECHANISM.LOW))
                                                .withTimeout(3)
                                                .beforeStarting(Commands.runOnce(drive::resetEncoders)));
        }

        /**
         * A simple auto routine that drives backward a specified distance, and then
         * stops.
         */
        public static Command scoreCubeMid(Drivetrain drive, Mechanism mech) {
                return Commands.sequence(
                                mech.setArmPreset(MECHANISM.MID)
                                                .withTimeout(2),

                                mech.launchGamePiece()
                                                .withTimeout(1),

                                drive.positionDriveCommand(-120, -120)
                                                .alongWith(mech.setArmPreset(MECHANISM.HOME))
                                                .withTimeout(5)
                                                .beforeStarting(Commands.runOnce(drive::resetEncoders)),

                                drive.positionDriveCommand(-32, 32)
                                                .alongWith(mech.setArmPreset(MECHANISM.LOW))
                                                .withTimeout(3)
                                                .beforeStarting(Commands.runOnce(drive::resetEncoders)));
        }

        /**
         * A simple auto routine that drives backward a specified distance, and then
         * stops.
         */
        public static Command scoreCubeHigh(Drivetrain drive, Mechanism mech) {
                return Commands.sequence(
                                mech.setArmPreset(MECHANISM.HIGH)
                                                .withTimeout(2),

                                mech.launchGamePiece()
                                                .withTimeout(1),

                                drive.positionDriveCommand(-120, -120)
                                                .alongWith(mech.setArmPreset(MECHANISM.HOME))
                                                .withTimeout(5)
                                                .beforeStarting(Commands.runOnce(drive::resetEncoders)),

                                drive.positionDriveCommand(-32, 32)
                                                .withTimeout(3)
                                                .beforeStarting(Commands.runOnce(drive::resetEncoders)));
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
                                drive.positionDriveCommand(-96, -96)
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
        public static Command scoreHighAndBalance(Drivetrain drive, Mechanism mech) {
                return Commands.sequence(
                                mech.setArmPreset(MECHANISM.HIGH)
                                                .withTimeout(2),

                                mech.launchGamePiece()
                                                .withTimeout(1),

                                drive.positionDriveCommand(-81, -81)
                                                .alongWith(mech.setArmPreset(MECHANISM.HOME))
                                                .withTimeout(5)
                                                .beforeStarting(Commands.runOnce(drive::resetEncoders)),

                                drive.balance());
        }

        /**
         * A highly sophisticated auto routine that places a gamepiece in the hybrid
         * zone,
         * drives back, and then drives to the charging station and balances.
         */
        public static Command mobilityAndBalance(Drivetrain drive, Mechanism mech) {
                return Commands.sequence(
                                mech.setArmPreset(MECHANISM.HIGH)
                                                .withTimeout(2),

                                mech.launchGamePiece()
                                                .withTimeout(1),

                                // Drive onto the charging station
                                drive.positionDriveCommand(-160, -160)
                                                .alongWith(mech.setArmPreset(MECHANISM.HOME))
                                                .withTimeout(7),

                                // Drive to gyro heading
                                drive.turnToGyro(0)
                                                .withTimeout(2),

                                // Drive onto the charging station
                                drive.positionDriveCommand(90, 90)
                                                .beforeStarting(Commands.runOnce(drive::resetEncoders))
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
                                drive.seekTarget().withTimeout(0.75),

                                drive.goToTarget(),

                                drive.seekTarget().withTimeout(0.75),

                                mech.setArmPreset(MECHANISM.MID).withTimeout(1.6),

                                mech.releaseGamePiece().withTimeout(0.5),

                                mech.setArmPreset(MECHANISM.HOME));
        }

        /**
         * An incomrehnsibly sophisticated auto routine that places a cube on the tope
         * platform, drives back onto and beyond the charging station, and then drives
         * back to charging station and balances.
         */
        public static Command superAuto(Drivetrain drive, Mechanism mech) {
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

                                // Drive onto the charging station
                                drive.positionDriveCommand(90, 90)
                                                .beforeStarting(Commands.runOnce(drive::resetEncoders))
                                                .withTimeout(2),

                                // Balance the robot
                                drive.balance())
                                .beforeStarting(Commands.runOnce(drive::resetEncoders))
                                .beforeStarting(Commands.runOnce(drive::resetGyro));
        }

        private Autos() {
                throw new UnsupportedOperationException("This is a utility class!");
        }
}