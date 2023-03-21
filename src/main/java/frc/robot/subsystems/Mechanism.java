// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MECHANISM;
import frc.robot.Constants.MISC;
import frc.robot.util.control.ArmPresets;
import frc.robot.util.control.SparkMaxPID;

public class Mechanism extends SubsystemBase {
  private final CANSparkMax shoulderMotor = new CANSparkMax(MECHANISM.SHOULDER_ID, MotorType.kBrushless);
  private final CANSparkMax wristMotor = new CANSparkMax(MECHANISM.WRIST_ID, MotorType.kBrushless);

  private final SparkMaxAbsoluteEncoder shoulderEncoder;
  private final SparkMaxAbsoluteEncoder wristEncoder;

  private final SparkMaxPID shoulderController;
  private final SparkMaxPID wristController;

  /** Creates a new Mechanism. */
  public Mechanism() {
    this.shoulderMotor.restoreFactoryDefaults();
    this.wristMotor.restoreFactoryDefaults();

    this.shoulderMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    this.wristMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    this.shoulderMotor.setSmartCurrentLimit(60, 20);
    this.wristMotor.setSmartCurrentLimit(40, 15);

    this.shoulderMotor.setInverted(false);
    this.wristMotor.setInverted(false);

    this.shoulderEncoder = this.shoulderMotor.getAbsoluteEncoder(Type.kDutyCycle);
    this.wristEncoder = this.wristMotor.getAbsoluteEncoder(Type.kDutyCycle);

    this.shoulderEncoder.setInverted(MECHANISM.SHOULDER_ENCODER_INVERTED);
    this.wristEncoder.setInverted(MECHANISM.WRIST_ENCODER_INVERTED);

    this.shoulderEncoder.setPositionConversionFactor(0);
    this.wristEncoder.setPositionConversionFactor(0);

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
    this.wristController.setMotionProfileType(AccelStrategy.kSCurve);
  }

  /**
   * Set robot arm to desired preset position
   * 
   * @param preset
   */
  public CommandBase setArmPreset(ArmPresets preset) {
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
    if (MISC.WITHIN_TOLERANCE(this.getShoulderPosition(), angle, MECHANISM.SHOULDER_TOLERANCE))
      return;

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
  }
}
