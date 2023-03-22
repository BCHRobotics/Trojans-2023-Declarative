// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.CHASSIS;
import frc.robot.Constants.MISC;
import frc.robot.Constants.PERIPHERALS;
import frc.robot.util.Gyro;
import frc.robot.util.control.PID;
import frc.robot.util.control.SparkMaxPID;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;
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

  private final DifferentialDrive drive;

  // Objects for gyroscope sensor fusion and balancing
  private Gyro gyro;
  private PID gyroPid;

  // Estimator
  private DifferentialDrivePoseEstimator estimator;
  private DifferentialDriveOdometry odometry;

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

    this.drive = new DifferentialDrive(this.frontLeftMotor, this.frontRightMotor);

    // Objects for balancing
    this.gyro = new Gyro(CHASSIS.GYRO_PORT);
    this.gyroPid = new PID(CHASSIS.GYRO_CONSTANTS);

    this.estimator = new DifferentialDrivePoseEstimator(CHASSIS.DRIVE_KINEMATICS, gyro.getRotation2d(),
        Units.inchesToMeters(getLeftPosition()), Units.inchesToMeters(getRightPosition()), new Pose2d());
    this.odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), Units.inchesToMeters(getLeftPosition()), Units.inchesToMeters(getRightPosition()));
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
      this.setMaxOutput(CHASSIS.DEFAULT_OUTPUT + (max.getAsDouble() * CHASSIS.OUTPUT_INTERVAL)
          - (min.getAsDouble() * CHASSIS.OUTPUT_INTERVAL));
      this.setOutput(fwd.getAsDouble(), rot.getAsDouble());
    }).beforeStarting(() -> this.drive.setDeadband(PERIPHERALS.CONTROLLER_DEADBAND))
        .beforeStarting(this::enableRampRate)
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf).withName("arcadeDrive");
  }

  /**
   * Sets drivetrain position in inches
   */
  public CommandBase positionDriveCommand(double leftPos, double rightPos) {
    return runEnd(() -> {
      this.setPosition(leftPos, rightPos);
    }, this::emergencyStop).beforeStarting(this.enableBrakeMode()).beforeStarting(this.disableRampRate())
        .beforeStarting(() -> this.setMaxOutput(1)).withName("positionDrive");
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
      this.setDriveOutput(this.gyroPid.getOutput());
    }).until(this::balanced).beforeStarting(this.enableBrakeMode()).beforeStarting(this.disableRampRate())
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
  }

  /**
   * Returns whether or not the robot is balanced
   */
  public boolean balanced() {
    return MISC.WITHIN_TOLERANCE(this.gyro.getPitch(), 0,
        CHASSIS.GYRO_TOLERANCE);
  }

  public CommandBase balance2() {
    return new PIDCommand(
        new PIDController(0.007, 0.001, 0),
        // Close the loop on the turn rate
        () -> this.gyro.getPitch(),
        // Setpoint is 0
        0,
        // Pipe the output to the turning controls
        (output) -> this.setDriveOutput(output),
        // Require the robot drive
        this).beforeStarting(this.enableBrakeMode()).beforeStarting(this.disableRampRate())
        .beforeStarting(() -> this.setMaxOutput(1))
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
  }

  /**
   * Returns a command that enables brake mode on the drivetrain.
   */
  public CommandBase enableBrakeMode() {
    return runOnce(() -> this.setBrakeMode(IdleMode.kBrake));
  }

  /**
   * Returns a command that release brake mode on the drivetrain.
   */
  public CommandBase releaseBrakeMode() {
    return runOnce(() -> this.setBrakeMode(IdleMode.kCoast));
  }

  public void setBrakeMode(IdleMode idleMode) {
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
  public CommandBase enableRampRate() {
    return runOnce(() -> this.setRampRate(true));
  }

  /**
   * Returns a command that disables ramp rate on the drivetrain.
   */
  public CommandBase disableRampRate() {
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
   * Sets the drivetrain's maximum percent output
   * 
   * @param maxOutput in percent decimal
   */
  public void setMaxOutput(double maxOutput) {
    this.drive.setMaxOutput(maxOutput);
    SmartDashboard.putNumber("Max Drive Speed %", maxOutput * 100);
  }

  /**
   * Returns a command that stops the drivetrain its tracks.
   */
  public CommandBase emergencyStop() {
    return run(() -> {
      this.frontLeftMotor.disable();
      this.frontRightMotor.disable();
      this.backLeftMotor.disable();
      this.backRightMotor.disable();
    }).beforeStarting(this::enableBrakeMode).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  /**
   * Sets motor output using arcade drive controls
   * 
   * @param forward linear motion [-1 --> 1] (Backwards --> Forward)
   * @param rot     rotational motion [-1 --> 1] (L --> R)
   */
  public void setOutput(double forward, double rot) {
    this.drive.arcadeDrive(forward, rot);
  }

  /**
   * Sets motor output using arcade drive controls
   * 
   * @param percent linear motion [-1 --> 1] (Backwards --> Forward)
   */
  public void setDriveOutput(double percent) {
    this.frontLeftMotor.set(percent);
    this.frontRightMotor.set(percent);
  }

  /**
   * Sets robot position in inches
   * 
   * @param left  position in inches
   * @param right position in inches
   */
  public void setPosition(double left, double right) {
    if (MISC.WITHIN_TOLERANCE(this.getLeftPosition(), left, CHASSIS.TOLERANCE)
        && MISC.WITHIN_TOLERANCE(this.getRightPosition(), right, CHASSIS.TOLERANCE))
      return;

    this.leftMotorController.setSmartPosition(left);
    this.rightMotorController.setSmartPosition(right);
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

  /**
   * Returns estimated position
   */
  public Pose2d getPose() {
    // return estimator.getEstimatedPosition();
    return odometry.getPoseMeters();
  }

  /**
   * Resets estimated position
   */
  public void resetPose(Pose2d pose) {
    // estimator.resetPosition(gyro.getRotation2d(), Units.inchesToMeters(getLeftPosition()),
    //     Units.inchesToMeters(getRightPosition()), pose);
    odometry.resetPosition(gyro.getRotation2d(), Units.inchesToMeters(getLeftPosition()), Units.inchesToMeters(getRightPosition()), pose);
      
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    frontLeftMotor.setVoltage(leftVolts);
    frontRightMotor.setVoltage(leftVolts);

    backLeftMotor.setVoltage(rightVolts);
    backRightMotor.setVoltage(rightVolts);

    this.drive.feed();
  }


  @Override
  public void periodic() {
    odometry.resetPosition(gyro.getRotation2d(), Units.inchesToMeters(getLeftPosition()),
        Units.inchesToMeters(getRightPosition()), estimator.getEstimatedPosition());

    // This method will be called once per scheduler run
    this.drive.feed();

    SmartDashboard.putNumber("Pitch", this.gyro.getPitch());
  }

  /**
   * Generates the trajectory command using a PathPlanner trajectory.
   * @return ramseteCommand
   */
  public Command trajectoryCommand(PathPlannerTrajectory trajectory) {
    PPRamseteCommand ramseteCommand = new PPRamseteCommand(
      trajectory, 
      this::getPose, 
      new RamseteController(),
      CHASSIS.DRIVE_KINEMATICS,
      this::tankDriveVolts,
      this
    );

    return ramseteCommand;
  }
}
