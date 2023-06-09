// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// Import required classes
import frc.robot.Constants.MECHANISM;
import frc.robot.Constants.MISC;
import frc.robot.util.control.ArmPresets;
import frc.robot.util.control.SparkMaxPID;

import java.util.function.BooleanSupplier;

// Import required libraries
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Mechanism extends SubsystemBase {
  private final CANSparkMax shoulderMotor = new CANSparkMax(MECHANISM.SHOULDER_ID, MotorType.kBrushless);
  private final CANSparkMax wristMotor = new CANSparkMax(MECHANISM.WRIST_ID, MotorType.kBrushless);
  private final CANSparkMax clawMotor = new CANSparkMax(MECHANISM.CLAW_ID, MotorType.kBrushless);

  private final SparkMaxAbsoluteEncoder shoulderEncoder;
  private final SparkMaxAbsoluteEncoder wristEncoder;

  private final SparkMaxPID shoulderController;
  private final SparkMaxPID wristController;

  private final DigitalOutput coneLED;
  private final DigitalOutput cubeLED;

  /** Creates a new Mechanism. */
  public Mechanism() {
    this.shoulderMotor.restoreFactoryDefaults();
    this.wristMotor.restoreFactoryDefaults();
    this.clawMotor.restoreFactoryDefaults();

    this.shoulderMotor.setIdleMode(IdleMode.kBrake);
    this.wristMotor.setIdleMode(IdleMode.kBrake);
    this.clawMotor.setIdleMode(IdleMode.kBrake);

    this.shoulderMotor.setSmartCurrentLimit(60, 20);
    this.wristMotor.setSmartCurrentLimit(60, 20);
    this.wristMotor.setSmartCurrentLimit(5, 5);

    this.shoulderMotor.setInverted(false);
    this.wristMotor.setInverted(false);
    this.clawMotor.setInverted(true);

    this.shoulderEncoder = this.shoulderMotor.getAbsoluteEncoder(Type.kDutyCycle);
    this.wristEncoder = this.wristMotor.getAbsoluteEncoder(Type.kDutyCycle);

    this.shoulderEncoder.setInverted(MECHANISM.SHOULDER_ENCODER_INVERTED);
    this.wristEncoder.setInverted(MECHANISM.WRIST_ENCODER_INVERTED);

    this.shoulderEncoder.setPositionConversionFactor(MECHANISM.SHOULDER_CONVERSION_FACTOR);
    this.wristEncoder.setPositionConversionFactor(MECHANISM.WRIST_CONVERSION_FACTOR);

    this.shoulderEncoder.setZeroOffset(MECHANISM.SHOULDER_ENCODER_OFFSET);
    this.wristEncoder.setZeroOffset(MECHANISM.WRIST_ENCODER_OFFSET);

    this.shoulderMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    this.shoulderMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

    this.shoulderMotor.setSoftLimit(SoftLimitDirection.kForward, MECHANISM.SHOULDER_LIMIT);
    this.shoulderMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) MECHANISM.SHOULDER_DEFAULT_OFFSET);

    this.wristMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    this.wristMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

    this.wristMotor.setSoftLimit(SoftLimitDirection.kForward, MECHANISM.WRIST_LIMIT);
    this.wristMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) MECHANISM.WRIST_DEFAULT_OFFSET);

    this.shoulderController = new SparkMaxPID(this.shoulderMotor, MECHANISM.SHOULDER_CONTROL_CONSTANTS);
    this.wristController = new SparkMaxPID(this.wristMotor, MECHANISM.WRIST_CONTROL_CONSTANTS);

    // this.wristController.pushConstantsToDashboard("Wrist");

    this.shoulderController.setFeedbackDevice(shoulderEncoder);
    this.wristController.setFeedbackDevice(wristEncoder);

    this.shoulderController.setPIDWrapping(false);
    this.wristController.setPIDWrapping(false);

    this.shoulderController.setMotionProfileType(AccelStrategy.kSCurve);
    this.wristController.setMotionProfileType(AccelStrategy.kSCurve);

    this.coneLED = new DigitalOutput(MISC.CONE_LED_PORT);
    this.cubeLED = new DigitalOutput(MISC.CUBE_LED_PORT);
  }

  /**
   * Set robot arm to desired preset position
   * 
   * @param preset
   */
  public Command setArmPreset(ArmPresets preset) {
    return run(() -> {
      this.setShoulderAngle(Math.toDegrees(
          Math.acos((MECHANISM.SHOULDER_HEIGHT - MECHANISM.WRIST_HEIGHT_OFFSET - preset.wristHeight)
              / MECHANISM.ARM_LENGTH)));
      this.setWristAngle(Math.toDegrees(
          Math.acos((MECHANISM.SHOULDER_HEIGHT - MECHANISM.WRIST_HEIGHT_OFFSET - preset.wristHeight)
              / MECHANISM.ARM_LENGTH))
          + MECHANISM.WRIST_PARALLEL_OFFSET + preset.wristOffset);
    }).withName("setArmPreset");
  }

  /**
   * Sets shoulder anlge in degrees from zero reference
   * 
   * @param angle
   */
  private void setShoulderAngle(double angle) {
    if (MISC.WITHIN_TOLERANCE(this.getShoulderPosition(), angle,
        MECHANISM.SHOULDER_TOLERANCE)) {
      if (angle <= MECHANISM.SHOULDER_DEFAULT_OFFSET)
        this.shoulderMotor.disable();
      return;
    }
    this.shoulderController.setSmartPosition(angle, MECHANISM.SHOULDER_DEFAULT_OFFSET, MECHANISM.SHOULDER_LIMIT);
  }

  /**
   * Sets wrist angle in degrees from zero reference
   * 
   * @param angle
   */
  private void setWristAngle(double angle) {
    this.wristController.setSmartPosition(angle, MECHANISM.WRIST_DEFAULT_OFFSET,
        this.getShoulderPosition() < MECHANISM.SHOUDLER_MAX_EXTENSION_LIMIT
            ? ((this.shoulderEncoder.getPosition() - MECHANISM.SHOULDER_DEFAULT_OFFSET)
                / (MECHANISM.SHOUDLER_MAX_EXTENSION_LIMIT - MECHANISM.SHOULDER_DEFAULT_OFFSET)
                * (MECHANISM.WRIST_LIMIT - MECHANISM.WRIST_PARALLEL_OFFSET)
                + MECHANISM.WRIST_PARALLEL_OFFSET + MECHANISM.WRIST_DEFAULT_OFFSET)
            : MECHANISM.WRIST_LIMIT);
    // TODO: Remember to add dynamic offset, don't hit ground!!!
  }

  /**
   * Runs intake claw to collect game piece until reached proximity
   * 
   * @return "Grab Game-Piece" Command
   */
  public Command grabGamePiece(BooleanSupplier kill) {
    return runEnd(() -> this.setClawSpeed(0.5), () -> this.setClawSpeed(0))
        .until(kill::getAsBoolean);
  }

  /**
   * Runs intake claw to release game piece, waits 2 seconds then stops the motor
   * 
   * @return "Release Game-Piece" Command
   */
  public Command releaseGamePiece() {
    return Commands.sequence(
        runOnce(() -> this.setClawSpeed(-1)),
        new WaitCommand(0.8),
        runOnce(() -> this.setClawSpeed(0)));
  }

  /**
   * Sets claw speed in percent output [-1 --> 1]
   */
  private void setClawSpeed(double speed) {
    this.clawMotor.set(speed);
  }

  /**
   * Blinks the Cone LED
   * 
   * @return "Blink Cone LED" Command
   */
  public Command blinkConeLED() {
    return Commands.repeatingSequence(
        this.setConeLED(true),
        new WaitCommand(MISC.BLINK_INTERVAL),
        this.setConeLED(false),
        new WaitCommand(MISC.BLINK_INTERVAL))
        .andThen(this.setConeLED(false));
  }

  /**
   * Blinks the Cube LED
   * 
   * @return "Blink Cube LED" Command
   */
  public Command blinkCubeLED() {
    return Commands.repeatingSequence(
        this.setCubeLED(true),
        new WaitCommand(MISC.BLINK_INTERVAL),
        this.setCubeLED(false),
        new WaitCommand(MISC.BLINK_INTERVAL))
        .andThen(this.setCubeLED(false));
  }

  /**
   * Toggles the Cone LED
   * 
   * @return "Toggle Cone LED" Command
   */
  public Command setConeLED(boolean state) {
    SmartDashboard.putBoolean("Cone Request", state);
    return runOnce(() -> this.coneLED.set(state));
  }

  /**
   * Toggles the Cube LED
   * 
   * @return "Toggle Cube LED" Command
   */
  public Command setCubeLED(boolean state) {
    SmartDashboard.putBoolean("Cube Request", state);
    return runOnce(() -> this.cubeLED.set(state));
  }

  /**
   * Gets shoulder absolute encoder position value
   * 
   * @return Shoulder Position
   */
  public double getShoulderPosition() {
    return this.shoulderEncoder.getPosition();
  }

  /**
   * Gets wrist absolute encoder position value
   * 
   * @return Wrist Position
   */
  public double getWristPosition() {
    return this.wristEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Game Piece", this.sensor.getProximity());
    // this.wristController.retrieveDashboardConstants();
  }
}
