// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// Import required modules
import frc.robot.Commands.Autos;
import frc.robot.Constants.MECHANISM;
import frc.robot.Constants.PERIPHERALS;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Mechanism;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.revrobotics.CANSparkMax.IdleMode;

// Import required libraries
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
  private final Command driveAuto = Autos.driveBack(drivetrain);
  private final Command balanceAuto = Autos.driveBackAndBalance(drivetrain);
  private final Command scoreAuto = Autos.scoreTwoPieces(drivetrain, mechanism);
  private final Command plannerAuto = Autos.followTrajectoryCommand(drivetrain,
      PathPlanner.loadPath("TestPathD", new PathConstraints(3, 1)), true);

  // A chooser for autonomous commands
  SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    // Set default commands
    // Control the drive with split-stick arcade controls
    drivetrain.setDefaultCommand(
        drivetrain.arcadeDriveCommand(
            () -> -driverController.getLeftY(), () -> -driverController.getRightX(),
            () -> driverController.getLeftTriggerAxis(), () -> driverController.getRightTriggerAxis())
            .beforeStarting(drivetrain.enableRampRate()));

    configureBindings();

    // Add commands to the autonomous command chooser
    autoChooser.setDefaultOption("Drive Back", driveAuto);
    autoChooser.addOption("Balance", balanceAuto);
    autoChooser.addOption("Score", scoreAuto);
    autoChooser.addOption("Planner", plannerAuto);
    autoChooser.addOption("ArmAuto", Autos.loadPathAutos(drivetrain, mechanism));

    SmartDashboard.putData("Autonomous Route", autoChooser);
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

    driverController.rightBumper().or(driverController.y())
        .onTrue(drivetrain.enableBrakeMode()).onFalse(drivetrain.releaseBrakeMode());

    driverController.y().whileTrue(drivetrain.balance());
    driverController.a().onTrue(Autos.goToAprilTag(drivetrain));

    driverController.leftBumper().whileTrue(
        drivetrain.enableBrakeMode().andThen(drivetrain.emergencyStop())).onFalse(drivetrain.releaseBrakeMode());

    operatorController.povUp().onTrue(mechanism.setArmPreset(MECHANISM.TOP));
    operatorController.povLeft().onTrue(mechanism.setArmPreset(MECHANISM.MID));
    operatorController.povDown().onTrue(mechanism.setArmPreset(MECHANISM.GROUND));
    operatorController.povRight().onTrue(mechanism.setArmPreset(MECHANISM.TRANSPORT));
    operatorController.rightStick().onTrue(mechanism.setArmPreset(MECHANISM.STATION));
    operatorController.leftStick().onTrue(mechanism.setArmPreset(MECHANISM.DEFAULT));
  }

  /**
   * HALTS all chassis motors
   */
  public void EMERGENCY_STOP() {
    this.drivetrain.setBrakeMode(IdleMode.kBrake);
    this.drivetrain.setDriveOutput(0);
  }

  /**
   * Resets arm to default position
   */
  public void ARM_RESET() {
    mechanism.setArmPreset(MECHANISM.DEFAULT).schedule();
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
