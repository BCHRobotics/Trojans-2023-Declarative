// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.CHASSIS;
import frc.robot.Constants.PATHING;
import frc.robot.Constants.PERIPHERALS;
import frc.robot.Constants.VISION;
import frc.robot.Constants.VISION.TARGET_TYPE;
import frc.robot.util.control.SparkMaxPID;
import frc.robot.util.devices.Gyro;
import frc.robot.util.devices.Limelight;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
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
  private final RelativeEncoder rightEncoder;

  private final SparkMaxPID leftMotorController;
  private final SparkMaxPID rightMotorController;

  private final DifferentialDrive drive;
  private final DifferentialDrivePoseEstimator driveOdometry;

  // Objects for gyroscope odometry and balancing
  private final Gyro gyro;
  private final Limelight limelight;

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
    this.rightEncoder = this.frontRightMotor.getEncoder();

    this.leftEncoder.setPositionConversionFactor(CHASSIS.LEFT_POSITION_CONVERSION);
    this.rightEncoder.setPositionConversionFactor(CHASSIS.RIGHT_POSITION_CONVERSION);

    this.leftEncoder.setVelocityConversionFactor(CHASSIS.LEFT_VELOCITY_CONVERSION);
    this.rightEncoder.setVelocityConversionFactor(CHASSIS.RIGHT_VELOCITY_CONVERSION);

    this.leftMotorController = new SparkMaxPID(this.frontLeftMotor, CHASSIS.LEFT_DRIVE_CONSTANTS);
    this.rightMotorController = new SparkMaxPID(this.frontRightMotor, CHASSIS.RIGHT_DRIVE_CONSTANTS);

    this.leftMotorController.setFeedbackDevice(this.leftEncoder);
    this.rightMotorController.setFeedbackDevice(this.rightEncoder);

    this.leftMotorController.setMotionProfileType(AccelStrategy.kTrapezoidal);
    this.rightMotorController.setMotionProfileType(AccelStrategy.kTrapezoidal);

    this.gyro = Gyro.getInstance();
    this.limelight = Limelight.getInstance();
    this.limelight.setDesiredTarget(TARGET_TYPE.APRILTAG);

    this.drive = new DifferentialDrive(this.frontLeftMotor, this.frontRightMotor);

    this.driveOdometry = new DifferentialDrivePoseEstimator(PATHING.DRIVE_KINEMATICS, this.gyro.getRotation2d(),
        Units.inchesToMeters(this.getLeftPosition()),
        Units.inchesToMeters(this.getRightPosition()), this.limelight.getPose2d());

  }

  /**
   * Returns a command that drives the robot with arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   * @param min the commanded snail percentage
   * @param max the commanded turbo percentage
   */
  public Command arcadeDriveCommand(DoubleSupplier fwd, DoubleSupplier rot,
      DoubleSupplier min, DoubleSupplier max) {
    return runOnce(() -> {
      this.setMaxOutput(CHASSIS.DEFAULT_OUTPUT + (max.getAsDouble() * CHASSIS.MAX_INTERVAL)
          - (min.getAsDouble() * CHASSIS.MIN_INTERVAL));
      this.arcadeDrive(fwd.getAsDouble(), rot.getAsDouble());
    })
        .beforeStarting(() -> this.drive.setDeadband(PERIPHERALS.CONTROLLER_DEADBAND))
        .beforeStarting(this::enableRampRate)
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf)
        .withName("arcadeDrive");
  }

  /**
   * Sets drivetrain position in inches
   */
  public Command positionDriveCommand(double leftPos, double rightPos) {
    return startEnd(() -> {
      this.setPosition(leftPos, rightPos);
    },
        this::emergencyStop)
        .until(this::reachedPosition)
        .beforeStarting(this::enableBrakeMode)
        .beforeStarting(this::disableRampRate)
        .withName("positionDrive");
  }

  /**
   * Returns whether or not the robot has reached the desired position
   */
  private boolean reachedPosition() {
    return this.leftMotorController.reachedSetpoint(this.getLeftPosition(), CHASSIS.TOLERANCE) &&
        this.rightMotorController.reachedSetpoint(this.getRightPosition(), CHASSIS.TOLERANCE);
  }

  /**
   * Uses PID along with limelight data to turn to target
   */
  public Command balance() {
    return new PIDCommand(
        new PIDController(
            CHASSIS.GYRO_CONSTANTS.kP,
            CHASSIS.GYRO_CONSTANTS.kI,
            CHASSIS.GYRO_CONSTANTS.kD),
        // Close the loop on the turn rate
        this.gyro::getPitch,
        // Setpoint is 0
        0,
        // Pipe the output to the turning controls
        (output) -> this.driveStraight(output),
        // Require the robot drive
        this)
        .andThen(this::emergencyStop)
        .beforeStarting(this::enableBrakeMode)
        .beforeStarting(this::disableRampRate);
  }

  public Command seekAprilTag() {
    return new PIDCommand(
        new PIDController(
            CHASSIS.SEEK_CONSTANTS.kP,
            CHASSIS.SEEK_CONSTANTS.kI,
            CHASSIS.SEEK_CONSTANTS.kD),
        // Close the loop on the turn rate
        this.limelight::getTargetX,
        // Setpoint is 0
        0,
        // Pipe the output to the turning controls
        (output) -> this.turn(output),
        // require the drivetrain
        this)
        .andThen(this::emergencyStop)
        .beforeStarting(this::enableBrakeMode)
        .beforeStarting(this::disableRampRate)
        .beforeStarting(this::resetEncoders);
  }

  /**
   * Uses smart-motion to turn to a limelight target
   */
  public Command seekTarget() {
    return startEnd(() -> {
      this.setPosition((this.limelight.getTargetX() * CHASSIS.TURNING_CONVERSION),
          -(this.limelight.getTargetX() * CHASSIS.TURNING_CONVERSION));
    },
        this::emergencyStop)
        .until(this.limelight::reachedTargetX)
        .beforeStarting(this::enableBrakeMode)
        .beforeStarting(this::disableRampRate)
        .beforeStarting(this::resetEncoders)
        .beforeStarting(this::searchForTape)
        .andThen(this::searchForTags)
        .withName("huntTarget");
  }

  /**
   * Uses smart-motion to drive towards a limelight target
   */
  public Command goToTarget() {
    return this.positionDriveCommand(this.limelight.getTargetDistance() - VISION.MID_ARM_OFFSET,
        this.limelight.getTargetDistance() - VISION.MID_ARM_OFFSET)
        .beforeStarting(this::searchForTape)
        .andThen(this::searchForTags);
  }

  /**
   * Returns a command that enables brake mode on the drivetrain.
   */
  public Command enableBrakeMode() {
    return runOnce(() -> this.setBrakeMode(IdleMode.kBrake));
  }

  /**
   * Returns a command that release brake mode on the drivetrain.
   */
  public Command releaseBrakeMode() {
    return runOnce(() -> this.setBrakeMode(IdleMode.kCoast));
  }

  /**
   * sets the chassis brake mode
   */
  private void setBrakeMode(IdleMode idleMode) {
    this.frontLeftMotor.setIdleMode(idleMode);
    this.frontRightMotor.setIdleMode(idleMode);
    this.backLeftMotor.setIdleMode(idleMode);
    this.backRightMotor.setIdleMode(idleMode);
    this.pushControllerUpdate();
    SmartDashboard.putBoolean("Brake Mode", idleMode == IdleMode.kBrake);
  }

  /**
   * Returns a command that enables ramp rate on the drivetrain.
   */
  public Command enableRampRate() {
    return runOnce(() -> this.setRampRate(true));
  }

  /**
   * Returns a command that disables ramp rate on the drivetrain.
   */
  public Command disableRampRate() {
    return runOnce(() -> this.setRampRate(false));
  }

  private void setRampRate(boolean state) {
    this.frontLeftMotor.setOpenLoopRampRate(state ? CHASSIS.RAMP_RATE : 0);
    this.frontRightMotor.setOpenLoopRampRate(state ? CHASSIS.RAMP_RATE : 0);
    this.backLeftMotor.setOpenLoopRampRate(state ? CHASSIS.RAMP_RATE : 0);
    this.backRightMotor.setOpenLoopRampRate(state ? CHASSIS.RAMP_RATE : 0);
    this.pushControllerUpdate();
    SmartDashboard.putBoolean("Ramping", state);
  }

  /**
   * Returns a command that stops the drivetrain its tracks.
   */
  public Command emergencyStop() {
    return startEnd(() -> {
      this.frontLeftMotor.disable();
      this.frontRightMotor.disable();
      this.backLeftMotor.disable();
      this.backRightMotor.disable();
    }, this::releaseBrakeMode)
        .beforeStarting(this::enableBrakeMode)
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  /**
   * Basically like e-stop command for disabled mode only
   */
  public void killSwitch() {
    this.frontLeftMotor.disable();
    this.frontRightMotor.disable();
    this.backLeftMotor.disable();
    this.backRightMotor.disable();
    this.setBrakeMode(IdleMode.kBrake);
  }

  /**
   * Sets motor output using arcade drive controls
   * 
   * @param forward linear motion [-1 --> 1] (Backwards --> Forwards)
   * @param rot     rotational motion [-1 --> 1] (Left --> Right)
   */
  private void arcadeDrive(double forward, double rot) {
    this.drive.arcadeDrive(forward, rot);
  }

  /**
   * Sets motor output using arcade drive controls
   * 
   * @param percent linear motion [-1 --> 1] (Backwards --> Forwards)
   */
  private void driveStraight(double percent) {
    this.frontLeftMotor.set(percent);
    this.frontRightMotor.set(percent);
  }

  /**
   * Sets motor output using arcade drive controls
   * 
   * @param percent rotational motion [-1 --> 1] (Left --> Right)
   */
  private void turn(double percent) {
    this.frontLeftMotor.set(percent);
    this.frontRightMotor.set(-percent);
  }

  /**
   * Sets the drivetrain's maximum percent output
   * 
   * @param maxOutput in percent decimal
   */
  private void setMaxOutput(double maxOutput) {
    this.drive.setMaxOutput(maxOutput);
    SmartDashboard.putNumber("Max Drive Speed %", maxOutput * 100);
  }

  /**
   * Sets robot position in inches
   * 
   * @param left  position in inches
   * @param right position in inches
   */
  private void setPosition(double left, double right) {
    this.leftMotorController.setSmartPosition(left);
    this.rightMotorController.setSmartPosition(right);
  }

  /**
   * Returns the drivetrain's left encoder position in inches
   * 
   * @return Left Position
   */
  private double getLeftPosition() {
    return this.leftEncoder.getPosition();
  }

  /**
   * Returns the drivetrain's right encoder position in inches
   * 
   * @return Right Position
   */
  private double getRightPosition() {
    return this.rightEncoder.getPosition();
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
  private double getLeftVelocity() {
    return this.leftEncoder.getVelocity();
  }

  /**
   * Returns the drivetrain's left encoder velocity in inches / second
   * 
   * @return Left Velocity
   */
  private double getRightVelocity() {
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
    this.rightEncoder.setPosition(0);
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

  /**
   * Sets the limelight target to search for reflective
   * tape.
   */
  private void searchForTape() {
    this.limelight.setDesiredTarget(TARGET_TYPE.REFLECTIVE_TAPE);
  }

  /**
   * Sets the limelight target to search for april tags.
   */
  private void searchForTags() {
    this.limelight.setDesiredTarget(TARGET_TYPE.APRILTAG);
  }

  // Path planning methods

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return this.driveOdometry.getEstimatedPosition();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(Units.inchesToMeters(this.getLeftVelocity()),
        Units.inchesToMeters(this.getRightVelocity()));
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    this.resetEncoders();
    this.driveOdometry.resetPosition(
        this.gyro.getRotation2d(), Units.inchesToMeters(this.getLeftPosition()),
        Units.inchesToMeters(this.getRightPosition()), pose);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void setVoltageOutput(double leftVolts, double rightVolts) {
    this.frontLeftMotor.setVoltage(leftVolts);
    this.frontRightMotor.setVoltage(rightVolts);
    this.drive.feed();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    this.gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return this.gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -this.gyro.getRate();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    this.driveOdometry.update(this.gyro.getRotation2d(),
        Units.inchesToMeters(this.getLeftPosition()),
        Units.inchesToMeters(this.getRightPosition()));

    if (this.limelight.getDesiredTarget() == TARGET_TYPE.APRILTAG && this.limelight.getTargetExists()) {
      this.driveOdometry.addVisionMeasurement(this.limelight.getPose2d(), Timer.getFPGATimestamp());
    }

    this.drive.feed();

    SmartDashboard.putNumber("Pitch", this.gyro.getPitch());
    SmartDashboard.putNumber("Left Position", this.getLeftPosition());
    SmartDashboard.putNumber("Right Position", this.getRightPosition());
  }
}
