// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CHASSIS;
import frc.robot.Constants.MISC;
import frc.robot.Constants.PERIPHERALS;
import frc.robot.util.Gyro;
import frc.robot.util.control.PID;
import frc.robot.util.control.SparkMaxPID;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

public class Drivetrain extends SubsystemBase {

  private final CANSparkMax frontLeftMotor;
  private final CANSparkMax frontRightMotor;
  private final CANSparkMax backLeftMotor;
  private final CANSparkMax backRightMotor;

  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder righEncoder;

  private final SparkMaxPID leftMotorController;
  private final SparkMaxPID rightMotorController;

  // The robot's drive
  private final DifferentialDrive drive;

  private double maxOutput;
  private IdleMode idleMode;
  private boolean ramp;

  // Objects for gyroscope sensor fusion and balancing
  private Gyro gyro;
  private PID gyroPid;

  /** Creates a new Drive subsystem. */
  public Drivetrain() {

    this.frontLeftMotor = new CANSparkMax(CHASSIS.FRONT_LEFT_ID, MotorType.kBrushless);
    this.frontRightMotor = new CANSparkMax(CHASSIS.FRONT_RIGHT_ID, MotorType.kBrushless);
    this.backLeftMotor = new CANSparkMax(CHASSIS.BACK_LEFT_ID, MotorType.kBrushless);
    this.backRightMotor = new CANSparkMax(CHASSIS.BACK_RIGHT_ID, MotorType.kBrushless);

    this.frontLeftMotor.restoreFactoryDefaults();
    this.frontRightMotor.restoreFactoryDefaults();
    this.backLeftMotor.restoreFactoryDefaults();
    this.backRightMotor.restoreFactoryDefaults();

    this.frontLeftMotor.setIdleMode(IdleMode.kCoast);
    this.frontRightMotor.setIdleMode(IdleMode.kCoast);
    this.backLeftMotor.setIdleMode(IdleMode.kCoast);
    this.backRightMotor.setIdleMode(IdleMode.kCoast);

    this.frontLeftMotor.setSmartCurrentLimit(60, 20);
    this.frontRightMotor.setSmartCurrentLimit(60, 20);
    this.backLeftMotor.setSmartCurrentLimit(60, 20);
    this.backRightMotor.setSmartCurrentLimit(60, 20);

    this.frontLeftMotor.setInverted(CHASSIS.INVERTED);
    this.frontRightMotor.setInverted(!CHASSIS.INVERTED);

    this.backLeftMotor.follow(this.frontLeftMotor);
    this.backRightMotor.follow(this.frontRightMotor);

    this.leftEncoder = this.frontLeftMotor.getEncoder();
    this.righEncoder = this.frontRightMotor.getEncoder();

    this.leftEncoder.setPositionConversionFactor(CHASSIS.LEFT_POSITION_CONVERSION);
    this.righEncoder.setPositionConversionFactor(CHASSIS.RIGHT_POSITION_CONVERSION);

    this.leftEncoder.setVelocityConversionFactor(CHASSIS.LEFT_VELOCITY_CONVERSION);
    this.righEncoder.setVelocityConversionFactor(CHASSIS.RIGHT_VELOCITY_CONVERSION);

    this.leftMotorController = new SparkMaxPID(this.frontLeftMotor, CHASSIS.LEFT_DRIVE_CONSTANTS);
    this.rightMotorController = new SparkMaxPID(this.frontRightMotor, CHASSIS.RIGHT_DRIVE_CONSTANTS);

    this.leftMotorController.setFeedbackDevice(this.leftEncoder);
    this.rightMotorController.setFeedbackDevice(this.righEncoder);

    this.leftMotorController.setMotionProfileType(AccelStrategy.kTrapezoidal);
    this.rightMotorController.setMotionProfileType(AccelStrategy.kTrapezoidal);

    this.drive = new DifferentialDrive(frontLeftMotor, frontRightMotor);

    // Objects for balancing
    this.gyro = new Gyro(CHASSIS.GYRO_PORT);
    this.gyroPid = new PID(CHASSIS.GYRO_CONSTANTS);

  }

  /**
   * Returns a command that drives the robot with arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   * @param min the commanded snail percentage
   * @param max the commanded turbo percentage
   */
  public CommandBase arcadeDriveCommand(DoubleSupplier fwd, DoubleSupplier rot, DoubleSupplier min,
      DoubleSupplier max) {
    // A split-stick arcade command, with forward/backward controlled by the left
    // hand, and turning controlled by the right.
    return run(() -> {
      this.maxOutput = (CHASSIS.DEFAULT_OUTPUT + (max.getAsDouble() * CHASSIS.OUTPUT_INTERVAL)
          - (min.getAsDouble() * CHASSIS.OUTPUT_INTERVAL));
      this.drive.arcadeDrive(fwd.getAsDouble(), rot.getAsDouble());
    }).beforeStarting(() -> this.drive.setDeadband(PERIPHERALS.CONTROLLER_DEADBAND))
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf).withName("arcadeDrive");
  }

  public void setOutput(double left, double right) {
    this.frontLeftMotor.set(left);
    this.frontRightMotor.set(right);
  }

  /**
   * Sets drivetrain position in inches
   * 
   * @param leftPos
   * @param rightPos
   */
  public CommandBase setPosition(double leftPos, double rightPos) {
    return run(() -> {
      this.ramp = true;
      this.leftMotorController.setSmartPosition(leftPos);
      this.rightMotorController.setSmartPosition(rightPos);
    }).until(() -> {
      return MISC.WITHIN_TOLERANCE(this.getLeftPosition(), leftPos, CHASSIS.TOLERANCE)
          && MISC.WITHIN_TOLERANCE(this.getRightPosition(), rightPos, CHASSIS.TOLERANCE);
    }).withName("positionDrive");
  }

  public CommandBase balance() {
    return run(() -> {
      /**
       * Uses PID to balance robot on charging station
       */
      this.gyroPid.setTarget(0);
      this.gyroPid.referenceTimer();
      this.gyroPid.setInput(this.gyro.getPitch());
      this.gyroPid.calculate();
      this.ramp = true;
      this.maxOutput = 1;
      this.frontLeftMotor.set(this.gyroPid.getOutput());
      this.frontRightMotor.set(this.gyroPid.getOutput());
    }).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
  }

  /**
   * Returns a command that enables brake mode on the drivetrain.
   * 
   * @param idleMode Brake vs Coast
   */
  public CommandBase setIdleMode(IdleMode idleMode) {
    // Motor idle mode set to braking
    return runOnce(() -> {
      this.idleMode = idleMode;
      this.frontLeftMotor.setIdleMode(idleMode);
      this.frontRightMotor.setIdleMode(idleMode);
      this.backLeftMotor.setIdleMode(idleMode);
      this.backRightMotor.setIdleMode(idleMode);
      this.pushControllerUpdate();
    });
  }

  /**
   * Returns a command that enables ramp rate on the drivetrain.
   * 
   * @param state true vs false
   */
  public CommandBase setRampRate(boolean state) {
    return runOnce(() -> {
      this.ramp = state;
      this.frontLeftMotor.setOpenLoopRampRate(state ? CHASSIS.RAMP_RATE : 0);
      this.frontRightMotor.setOpenLoopRampRate(state ? CHASSIS.RAMP_RATE : 0);
      this.backLeftMotor.setOpenLoopRampRate(state ? CHASSIS.RAMP_RATE : 0);
      this.backRightMotor.setOpenLoopRampRate(state ? CHASSIS.RAMP_RATE : 0);
      this.pushControllerUpdate();
    });
  }

  /**
   * Returns a command that stops the drivetrain its tracks.
   */
  public CommandBase emergencyStop() {
    // Motor idle mode set to braking
    return run(() -> {
      this.frontLeftMotor.disable();
      this.frontRightMotor.disable();
      this.backLeftMotor.disable();
      this.backRightMotor.disable();
    }).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  /**
   * Returns the drivetrain's left encoder position in inches
   * 
   * @return Left Position
   */
  public double getLeftPosition() {
    return this.leftEncoder.getPosition();
  }

  /**
   * Returns the drivetrain's right encoder position in inches
   * 
   * @return Right Position
   */
  public double getRightPosition() {
    return this.righEncoder.getPosition();
  }

  /**
   * Returns the drivetrain's average encoder position in inches
   * 
   * @return Average Position
   */
  public double getAveragePosition() {
    return (this.getLeftPosition() + this.getRightPosition()) / 2;
  }

  /**
   * Returns the drivetrain's left encoder velocity in inches / second
   * 
   * @return Left Velocity
   */
  public double getLeftVelocity() {
    return this.leftEncoder.getVelocity();
  }

  /**
   * Returns the drivetrain's left encoder velocity in inches / second
   * 
   * @return Left Velocity
   */
  public double getRightVelocity() {
    return this.leftEncoder.getVelocity();
  }

  /**
   * Returns the drivetrain's average encoder velocty in inches / second
   * 
   * @return Average Velocity
   */
  public double getAverageVelocity() {
    return (this.getLeftVelocity() + this.getRightVelocity()) / 2;
  }

  /**
   * Resest all drivetrain encoder positions
   */
  public void resetEncoders() {
    this.leftEncoder.setPosition(0);
    this.righEncoder.setPosition(0);
  }

  /**
   * Updates motor controllers after settings change
   */
  private void pushControllerUpdate() {
    this.frontLeftMotor.burnFlash();
    this.frontRightMotor.burnFlash();
    this.backLeftMotor.burnFlash();
    this.backRightMotor.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    this.drive.feed();

    this.setIdleMode(this.idleMode);
    this.setRampRate(this.ramp);
    this.drive.setMaxOutput(this.maxOutput);

    SmartDashboard.putNumber("Max Drive Speed %", this.maxOutput * 100);
    SmartDashboard.putBoolean("Brake Mode", this.idleMode == IdleMode.kBrake ? true : false);
    SmartDashboard.putNumber("Pitch", this.gyro.getPitch());
  }
}
