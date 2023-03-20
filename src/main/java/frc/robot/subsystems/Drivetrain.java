// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Drivetrain extends SubsystemBase {

  private final CANSparkMax frontLeftMotor = new CANSparkMax(DriveConstants.FRONT_LEFT_ID, MotorType.kBrushless);
  private final CANSparkMax frontRightMotor = new CANSparkMax(DriveConstants.FRONT_RIGHT_ID, MotorType.kBrushless);
  private final CANSparkMax backLeftMotor = new CANSparkMax(DriveConstants.BACK_LEFT_ID, MotorType.kBrushless);
  private final CANSparkMax backRightMotor = new CANSparkMax(DriveConstants.BACK_RIGHT_ID, MotorType.kBrushless);

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(frontLeftMotor, frontRightMotor);

  /** Creates a new Drive subsystem. */
  public Drivetrain() {

    this.frontLeftMotor.restoreFactoryDefaults();
    this.frontRightMotor.restoreFactoryDefaults();
    this.backLeftMotor.restoreFactoryDefaults();
    this.backRightMotor.restoreFactoryDefaults();

    this.frontLeftMotor.setIdleMode(IdleMode.kCoast);
    this.frontRightMotor.setIdleMode(IdleMode.kCoast);
    this.backLeftMotor.setIdleMode(IdleMode.kCoast);
    this.backRightMotor.setIdleMode(IdleMode.kCoast);

    this.backLeftMotor.follow(frontLeftMotor);
    this.backRightMotor.follow(backRightMotor);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    this.frontRightMotor.setInverted(true);
  }

  /**
   * Returns a command that drives the robot with arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   * @param min the commanded snail percentage
   * @param max the commanded tubro percentage
   */
  public CommandBase arcadeDriveCommand(DoubleSupplier fwd, DoubleSupplier rot, DoubleSupplier min,
      DoubleSupplier max) {
    // A split-stick arcade command, with forward/backward controlled by the left
    // hand, and turning controlled by the right.
    return run(() -> {
      double multiplier = (DriveConstants.DEFAULT_OUTPUT + (max.getAsDouble() * DriveConstants.OUTPUT_INTERVAL)
          - (min.getAsDouble() * DriveConstants.OUTPUT_INTERVAL));
      m_drive.arcadeDrive(
          fwd.getAsDouble() * multiplier, rot.getAsDouble() * multiplier);
      SmartDashboard.putNumber("Max Drive Speed %", multiplier * 100);
    })
        .withName("arcadeDrive");
  }

  /**
   * Returns a command that enables brake mode on the drivetrain.
   */
  public CommandBase setBrakeMode(IdleMode idleMode) {
    // Motor idle mode set to braking
    return runOnce(() -> {
      this.frontLeftMotor.setIdleMode(idleMode);
      this.frontRightMotor.setIdleMode(idleMode);
      this.backLeftMotor.setIdleMode(idleMode);
      this.backRightMotor.setIdleMode(idleMode);
      this.pushControllerUpdate();
      SmartDashboard.putBoolean("Brake Mode", idleMode == IdleMode.kBrake ? true : false);
    });
  }

  private void pushControllerUpdate() {
    this.frontLeftMotor.burnFlash();
    this.frontRightMotor.burnFlash();
    this.backLeftMotor.burnFlash();
    this.backRightMotor.burnFlash();
  }

}
