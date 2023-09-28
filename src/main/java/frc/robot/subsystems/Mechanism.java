// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// Import required classes
import frc.robot.Constants.MECHANISM;
import frc.robot.Constants.MISC;
import frc.robot.util.control.ArmPresets;
import frc.robot.util.control.SparkMaxPID;

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
    this.clawMotor.setSmartCurrentLimit(40, 20);

    this.shoulderMotor.setInverted(false);
    this.wristMotor.setInverted(false);
    this.clawMotor.setInverted(true);

    this.clawMotor.setOpenLoopRampRate(0.1);
    this.clawMotor.enableVoltageCompensation(12);

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

    this.shoulderController.setFeedbackDevice(shoulderEncoder);
    this.wristController.setFeedbackDevice(wristEncoder);

    this.shoulderController.setPIDWrapping(false);
    this.wristController.setPIDWrapping(false);

    this.shoulderController.setMotionProfileType(AccelStrategy.kSCurve);
    this.wristController.setMotionProfileType(AccelStrategy.kTrapezoidal);

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
      SmartDashboard.putString("Arm Preset", preset.name);
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
    this.wristController.setSmartPosition(angle, MECHANISM.WRIST_DEFAULT_OFFSET, MECHANISM.WRIST_LIMIT);
  }

  /**
   * Runs intake claw to collect game piece until reached proximity
   * 
   * @return "Grab Game-Piece" Command
   */
  public Command grabCube() {
    return startEnd(() -> this.setClawSpeed(0.4), () -> this.setClawSpeed(0.02))
        .until(() -> this.gamePieceDetected(MISC.CUBE_DETECTION_CURRENT))
        .andThen(
            () -> SmartDashboard.putBoolean("Game Piece", true));
  }

  /**
   * Runs intake claw to collect game piece until reached proximity
   * 
   * @return "Grab Game-Piece" Command
   */
  public Command grabCone() {
    return startEnd(() -> this.setClawSpeed(0.7), () -> this.setClawSpeed(0.02))
        .until(() -> this.gamePieceDetected(MISC.CONE_DETECTION_CURRENT))
        .andThen(
            () -> SmartDashboard.putBoolean("Game Piece", true));
  }

  /**
   * Returns whether or not a game piece was detected
   */
  private boolean gamePieceDetected(double currentLimit) {
    return this.clawMotor.getOutputCurrent() >= currentLimit;
  }

  /**
   * Runs intake claw to release game piece, waits 0.8 seconds then stops the
   * motor
   * 
   * @return "Release Game-Piece" Command
   */
  public Command releaseGamePiece() {
    return startEnd(
        () -> this.setClawSpeed(-0.35),
        () -> this.setClawSpeed(0))
        .withTimeout(0.8)
        .beforeStarting(() -> SmartDashboard.putBoolean("Game Piece", false));
  }

  /**
   * Runs intake claw to launch game piece, waits 0.6 seconds then stops the
   * motor
   * 
   * @return "Launch Game-Piece" Command
   */
  public Command launchGamePiece() {
    return startEnd(
        () -> this.setClawSpeed(-1),
        () -> this.setClawSpeed(0))
        .withTimeout(0.8)
        .beforeStarting(() -> SmartDashboard.putBoolean("Game Piece", false));
  }

  /**
   * Disables intake claw motor
   * 
   * @return
   */
  public Command disableClaw() {
    return runOnce(this.clawMotor::disable)
        .beforeStarting(() -> SmartDashboard.putBoolean("Game Piece", false));
  }

  /**
   * Sets claw speed in percent output [-1 --> 1]
   */
  private void setClawSpeed(double speed) {
    this.clawMotor.set(speed);
  }

  public void shutDown() {
    this.setClawSpeed(0);
  }

  /**
   * Blinks the Cone LED
   * 
   * @return "Blink Cone LED" Command
   */
  public Command blinkConeLED() {
    return Commands.sequence(
        this.setConeLED(true),
        new WaitCommand(MISC.BLINK_INTERVAL),
        this.setConeLED(false),
        new WaitCommand(MISC.BLINK_INTERVAL))
        .repeatedly();
  }

  /**
   * Blinks the Cube LED
   * 
   * @return "Blink Cube LED" Command
   */
  public Command blinkCubeLED() {
    return Commands.sequence(
        this.setCubeLED(true),
        new WaitCommand(MISC.BLINK_INTERVAL),
        this.setCubeLED(false),
        new WaitCommand(MISC.BLINK_INTERVAL))
        .repeatedly();
  }

  /**
   * Toggles the Cone LED
   * 
   * @return "Toggle Cone LED" Command
   */
  public Command setConeLED(boolean state) {
    return runOnce(() -> this.coneLED.set(state));
  }

  /**
   * Toggles the Cube LED
   * 
   * @return "Toggle Cube LED" Command
   */
  public Command setCubeLED(boolean state) {
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

    SmartDashboard.putBoolean("Cube Request", this.cubeLED.get());
    SmartDashboard.putBoolean("Cone Request", this.coneLED.get());
  }
}
