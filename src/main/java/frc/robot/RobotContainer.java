// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Commands.Autos;
import frc.robot.Constants.CHASSIS;
import frc.robot.Constants.MECHANISM;
import frc.robot.Constants.PERIPHERALS;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Mechanism;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.RamseteAutoBuilder;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final Drivetrain drivetrain = new Drivetrain();
  private final Mechanism mechanism = new Mechanism();

  // The driver's controller
  CommandXboxController driverController = new CommandXboxController(PERIPHERALS.DRIVER_PORT);
  CommandXboxController operatorController = new CommandXboxController(PERIPHERALS.OPERATOR_PORT);

  // The autonomous routines
  // A simple auto routine that drives backwards a specified distance, and then
  // stops.
  private final Command simpleAuto = Autos.simpleAuto(drivetrain);
  // A complex auto routine that drives backwards, then balances the robot
  private final Command complexAuto = Autos.complexAuto(drivetrain);

  // Pathplanner's auto path generator && Map of locations to commands;
  private final RamseteAutoBuilder autoBuilder;
  private final Map<String, Command> autoMap = new HashMap<>();
  // A chooser for autonomous commands
  SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    // Set default commands
    // Control the drive with split-stick arcade controls
    drivetrain.setDefaultCommand(
        drivetrain.arcadeDriveCommand(
            () -> -driverController.getLeftY(), () -> -driverController.getRightX(),
            () -> driverController.getLeftTriggerAxis(), () -> driverController.getRightTriggerAxis())
            .beforeStarting(() -> {
              drivetrain.setRampRate(true);
              drivetrain.setIdleMode(IdleMode.kBrake);
            }));

    configureBindings();

    // Initialize PathPlanner generator
    autoBuilder = new RamseteAutoBuilder(drivetrain::getPose, drivetrain::resetPose, new RamseteController(), CHASSIS.driveKinematics, drivetrain::tankDriveVolts, autoMap, drivetrain);
    
    // Add commands to the autonomous command chooser
    autoChooser.setDefaultOption("Simple Auto", simpleAuto);
    autoChooser.addOption("Complex Auto", complexAuto);

    autoChooser.addOption("PathPlanner Auto", complexAuto);

    SmartDashboard.putData("Autonomous Route", autoBuilder.fullAuto(PathPlanner.loadPath("Circle", new PathConstraints(3, 1.5))));
  }

  /**
   * Use this method to define bindings between conditions and commands. These are
   * useful for
   * automating robot behaviors based on button and sensor input.
   *
   * <p>
   * Should be called during {@link Robot#robotInit()}.
   *
   * <p>
   * Event binding methods are available on the {@link Trigger} class.
   */
  public void configureBindings() {

    driverController.rightBumper().or(driverController.leftBumper().or(driverController.y()))
        .whileTrue(drivetrain.setIdleMode(IdleMode.kBrake))
        .whileFalse(drivetrain.setIdleMode(IdleMode.kCoast));

    driverController.y().whileTrue(drivetrain.balance());

    driverController.leftBumper().whileTrue(drivetrain.emergencyStop())
        .onFalse(drivetrain.setIdleMode(IdleMode.kCoast));

    operatorController.povUp().onTrue(mechanism.setArmPreset(MECHANISM.TOP));
    operatorController.povLeft().onTrue(mechanism.setArmPreset(MECHANISM.MID));
    operatorController.povRight().onTrue(mechanism.setArmPreset(MECHANISM.TRANSPORT));
    operatorController.povDown().onTrue(mechanism.setArmPreset(MECHANISM.GROUND));
    operatorController.rightStick().onTrue(mechanism.setArmPreset(MECHANISM.STATION));
    operatorController.leftStick().onTrue(mechanism.setArmPreset(MECHANISM.DEFAULT));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
