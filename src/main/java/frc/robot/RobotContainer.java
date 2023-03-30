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

// Import required libraries
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

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

  // The Autonomous trajectory path to follow
  private final PathPlannerTrajectory followMap = PathPlanner.loadPath("TestPathD", new PathConstraints(3, 1));

  // This is just an example event map. It would be better to have a constant,
  // global event map
  // in your code that will be used by all path following commands.
  private final HashMap<String, Command> eventMap = new HashMap<>();

  // The autonomous routines
  private final Command driveAuto = Autos.driveBack(drivetrain);
  private final Command balanceAuto = Autos.driveBackAndBalance(drivetrain);
  private final Command scoreAuto = Autos.scoreTwoPieces(drivetrain, mechanism);
  private final Command scoreAndBalance = Autos.scoreAndBalance(drivetrain, mechanism);
  private final Command plannerAuto = Autos.followTrajectoryCommand(drivetrain,
      this.followMap, true);

  private final FollowPathWithEvents advancedAuto = new FollowPathWithEvents(
      this.plannerAuto,
      this.followMap.getMarkers(),
      this.eventMap);

  // A chooser for autonomous commands
  SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {

    // Initialize autonomous eventmap options
    eventMap.put("step1", this.mechanism.setConeLED(true));
    eventMap.put("step2", this.mechanism.setCubeLED(true));

    // Set default commands

    // Control the drive with split-stick arcade controls
    this.drivetrain.setDefaultCommand(
        this.drivetrain.arcadeDriveCommand(
            () -> -this.driverController.getLeftY(), () -> -this.driverController.getRightX(),
            () -> this.driverController.getLeftTriggerAxis(), () -> this.driverController.getRightTriggerAxis()));

    configureBindings();

    // Add commands to the autonomous command chooser
    this.autoChooser.setDefaultOption("Drive Back", driveAuto);
    this.autoChooser.addOption("Balance", balanceAuto);
    this.autoChooser.addOption("Score", scoreAuto);
    this.autoChooser.addOption("Score and Balance", scoreAndBalance);
    this.autoChooser.addOption("Planner", plannerAuto);
    this.autoChooser.addOption("Advanced", advancedAuto);

    SmartDashboard.putData("Autonomous Route", this.autoChooser);
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

    // Driver braking and emergency stop controls
    this.driverController.rightBumper().or(this.driverController.y())
        .onTrue(this.drivetrain.enableBrakeMode()).onFalse(this.drivetrain.releaseBrakeMode());

    this.driverController.leftBumper()
        .whileTrue(this.drivetrain.enableBrakeMode()
            .andThen(this.drivetrain.emergencyStop()))
        .onFalse(this.drivetrain.releaseBrakeMode());

    // Driver automated routines
    this.driverController.a().whileTrue(this.drivetrain.seekAprilTag());
    this.driverController.y().whileTrue(this.drivetrain.balance());
    this.driverController.b().onTrue(Commands.runOnce(this.drivetrain::resetEncoders));

    // Operator arm preset controls
    this.operatorController.povUp().onTrue(this.mechanism.setArmPreset(MECHANISM.TOP));
    this.operatorController.povUpLeft().onTrue(this.mechanism.setArmPreset(MECHANISM.STATION));
    this.operatorController.povLeft().onTrue(this.mechanism.setArmPreset(MECHANISM.MID));
    this.operatorController.povDown().onTrue(this.mechanism.setArmPreset(MECHANISM.GROUND));
    this.operatorController.povDownRight().onTrue(this.mechanism.setArmPreset(MECHANISM.TRANSPORT));
    this.operatorController.povRight().onTrue(this.mechanism.setArmPreset(MECHANISM.DEFAULT));

    // Operator intake claw controls
    this.operatorController.x().onTrue(this.mechanism.grabGamePiece());
    this.operatorController.a().onTrue(this.mechanism.releaseGamePiece());
    this.operatorController.b().onTrue(this.mechanism.disableClaw());

    // Operator game piece signals
    this.operatorController.leftBumper().whileTrue(this.mechanism.blinkCubeLED());
    this.operatorController.rightBumper().whileTrue(this.mechanism.blinkConeLED());
  }

  /**
   * HALTS all chassis motors
   */
  public void EMERGENCY_STOP() {
    this.drivetrain.killSwitch();
  }

  /**
   * Resets arm to default position
   */
  public Command ARM_RESET() {
    return this.mechanism.setArmPreset(MECHANISM.DEFAULT);
  }

  /**
   * Resets chassis state
   */
  public Command CHASSIS_RESET() {
    return this.drivetrain.releaseBrakeMode();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return this.autoChooser.getSelected();
  }
}
