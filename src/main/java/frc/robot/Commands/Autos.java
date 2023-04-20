// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Drivetrain;

/** Container for auto command factories. */
public final class Autos {

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

        private Autos() {
                throw new UnsupportedOperationException("This is a utility class!");
        }
}