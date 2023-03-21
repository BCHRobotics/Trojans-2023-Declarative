// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.Drivetrain;

/** Container for auto command factories. */
public final class Autos {
  /**
   * A simple auto routine that drives forward a specified distance, and then
   * stops.
   */
  public static Command simpleAuto(Drivetrain drive) {
    return new FunctionalCommand(
        // Reset encoders on command start
        drive::resetEncoders,
        // Drive forward while the command is executing
        () -> drive.setPosition(-110, -110),
        // Stop driving at the end of the command
        interrupt -> drive.emergencyStop(),
        // End the command when the robot's driven distance exceeds the desired value
        () -> drive.getLeftPosition() <= -110,
        // Require the drive subsystem
        drive).alongWith(drive.setIdleMode(IdleMode.kBrake));
  }

  /**
   * A complex auto routine that drives forward, drops a hatch, and then drives
   * backward.
   */
  public static Command complexAuto(Drivetrain drive) {
    return Commands.sequence(
        // Drive forward up to the front of the cargo ship
        new FunctionalCommand(
            // Reset encoders on command start
            drive::resetEncoders,
            // Drive forward while the command is executing
            () -> drive.setPosition(-110, -110),
            // Stop driving at the end of the command
            interrupt -> drive.emergencyStop().alongWith(drive.setIdleMode(IdleMode.kBrake)),
            // End the command when the robot's driven distance exceeds the desired value
            () -> drive.getAveragePosition() <= -110,
            // Require the drive subsystem
            drive),

        // Balance the robot
        drive.balance().andThen(drive.setPosition(30, -30).beforeStarting(drive::resetEncoders)));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}