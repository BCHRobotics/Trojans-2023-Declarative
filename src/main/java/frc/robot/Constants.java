// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.util.control.ArmPresets;
import frc.robot.util.control.PIDConstants;
import frc.robot.util.control.SparkMaxConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class CHASSIS {
    public static final int FRONT_LEFT_ID = 10;
    public static final int FRONT_RIGHT_ID = 11;
    public static final int BACK_LEFT_ID = 12;
    public static final int BACK_RIGHT_ID = 13;

    // Drivetrain restrictions
    public static final double DEFAULT_OUTPUT = 0.75;
    public static final double OUTPUT_INTERVAL = 0.25;
    public static final double RAMP_RATE = 0.15; // s
    public static final double TOLERANCE = 1; // in
    public static final boolean INVERTED = false;

    // Chassis dimensions needed
    public static final double WHEEL_DIAMETER = 6;
    public static final double TRACK_WIDTH = 19;

    // Chasis conversion factors TODO: Re-collect conversion data
    public static final double LEFT_POSITION_CONVERSION = 48 / 18.23804473876953; // inches per
    // revolutions
    public static final double RIGHT_POSITION_CONVERSION = 48 / 18.14280891418457; // #inches / #revs

    public static final double LEFT_VELOCITY_CONVERSION = LEFT_POSITION_CONVERSION / 60.0; // inches per
                                                                                           // second
    public static final double RIGHT_VELOCITY_CONVERSION = RIGHT_POSITION_CONVERSION / 60.0; // #inches /
                                                                                             // 1 sec

    // input diameter = Δd inches between center wheels ~~v~~ in inches / degree
    public static final double TURNING_CONVERSION = (TRACK_WIDTH * Math.PI) / 360;

    // Drive PID Constants TODO: Re-tune Drivetrain PID
    public static final SparkMaxConstants LEFT_DRIVE_CONSTANTS = new SparkMaxConstants(
        0.00012, 0, 0.0025, 0, 0.00005, -1, 1, 0, 0, 6000, 2000, 0.2);
    public static final SparkMaxConstants RIGHT_DRIVE_CONSTANTS = new SparkMaxConstants(
        0.00012, 0, 0.0025, 0, 0.00005, -1, 1, 0, 0, 6000, 2000, 0.2);

    // Gyro constants
    public static final SerialPort.Port GYRO_PORT = SerialPort.Port.kMXP;
    public static final boolean GYRO_OUTPUT_INVERTED = false;
    public static final double GYRO_TOLERANCE = 0.8;

    // Gyro PID Constants TODO: Re-tune gyro
    public static final PIDConstants GYRO_CONSTANTS = new PIDConstants(0.007, 0.001, 0, 0);

    // Target seek PID Constants TODO: Tune seeking constants
    public static final PIDConstants SEEK_CONSTANTS = new PIDConstants(0.00012, 0, 0.0025, 0.00005);

    // Drive kinematics
    public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(
        Units.inchesToMeters(TRACK_WIDTH)); // inches to metres;
    public static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(1, 0.5);
  }

  public static final class MECHANISM {
    // Robot arm CAN IDs
    public static final int SHOULDER_ID = 20;
    public static final int WRIST_ID = 21;

    // Robot dimensions (inches)
    public static final double SHOULDER_HEIGHT = 36.75;
    public static final double ARM_LENGTH = 36;
    public static final double WRIST_HEIGHT_OFFSET = 2.25;
    public static final double FOREARM_LENGTH = 15;

    // Robot arm conversion factors
    public static final double SHOULDER_CONVERSION_FACTOR = 360; // Convert revs to degrees
    public static final double WRIST_CONVERSION_FACTOR = 360; // Convert revs to degrees
    public static final double WRIST_PARALLEL_OFFSET = 90;
    public static final double SHOULDER_DEFAULT_OFFSET = 17;
    public static final double WRIST_DEFAULT_OFFSET = SHOULDER_DEFAULT_OFFSET;
    public static final float SHOULDER_LIMIT = 130 + (float) MECHANISM.SHOULDER_DEFAULT_OFFSET;
    public static final float WRIST_LIMIT = 200 + (float) MECHANISM.WRIST_DEFAULT_OFFSET;
    public static final double SHOUDLER_MAX_EXTENSION_LIMIT = 55;
    public static final double SHOULDER_TOLERANCE = 1;

    // Robot arm ABSOLUTE encoder inversions
    public static final boolean SHOULDER_ENCODER_INVERTED = false;
    public static final boolean WRIST_ENCODER_INVERTED = false;

    // Robot arm ABSOLUTE encoder offset
    public static final double SHOULDER_ENCODER_OFFSET = (204.7492790) - MECHANISM.SHOULDER_DEFAULT_OFFSET;
    public static final double WRIST_ENCODER_OFFSET = (172.0870650) - MECHANISM.WRIST_DEFAULT_OFFSET;

    // Mechanism PID Constants TODO: Re-tune after modified wrist installation
    public static final SparkMaxConstants SHOULDER_CONTROL_CONSTANTS = new SparkMaxConstants(
        0.00014028, 0, 0.00051398, 0, 2e-6, -0.4, 1,
        0, 0, 5700, 3500, 0.2);

    public static final SparkMaxConstants WRIST_CONTROL_CONSTANTS = new SparkMaxConstants(
        2.1028E-05, 0, 5.1398E-05, 0, 0.00004, -1, 0.5,
        0, 0, 5700, 4500, 0.05);

    // Arm preset profiles TODO: Verify Presets with Drive Team
    public static final ArmPresets DEFAULT = new ArmPresets(0, -90);
    public static final ArmPresets TRANSPORT = new ArmPresets(0, -70);
    public static final ArmPresets GROUND = new ArmPresets(0, 0);
    public static final ArmPresets MID = new ArmPresets(38, 0);
    public static final ArmPresets STATION = new ArmPresets(41, 0);
    public static final ArmPresets TOP = new ArmPresets(52, 0);

    public static final ArmPresets[] PRESETS = { DEFAULT, TRANSPORT, GROUND, MID, STATION, TOP };
  }

  public static final class PERIPHERALS {
    public static final int DRIVER_PORT = 0;
    public static final int OPERATOR_PORT = 1;
    public static final double CONTROLLER_DEADBAND = 0.1;
  }

  public static final class MISC {

    // Game piece actuator presets in degrees
    public static final double CUBE_PRESET = 0.7;
    public static final double CONE_PRESET = 1;

    public static final int CUBE_LED_PORT = 1;
    public static final int CONE_LED_PORT = 0;

    public static final int BLINK_INTERVAL = 500; // milliseconds

    // Limelight vision constants
    public static final double LIMELIGHT_ANGLE = 21.5; // degrees
    public static final double LIMELIGHT_HEIGHT = 91.25; // inches
    public static final double LIMELIGHT_TOLERANCE = 0.5; // degrees (x axis)
    public static final double LIMELIGHT_CHASSIS_OFFSET = 16; // inches
    public static final double CUBE_TARGET_HEIGHT = 4.75; // inches
    public static final double APRILTAG_HEIGHT = 18.125; // inches

    // CSV Test version
    public static final int VERSION = 2;
    public static final String TEACH_MODE_FILE_NAME = "LEAD_BY_NOSE" + "_" + VERSION;

    // Autonomous directory
    public static final String ROOT_DIRECTORY = "csv/"; // "csv/";

    public static final double ENSURE_RANGE(double value, double min, double max) {
      return Math.min(Math.max(value, min), max);
    }

    public static final boolean IN_RANGE(double value, double min, double max) {
      return (value >= min) && (value <= max);
    }

    public static final boolean WITHIN_TOLERANCE(double value, double tolerance) {
      return (value >= -tolerance) && (value <= tolerance);
    }

    public static final boolean WITHIN_TOLERANCE(double input, double setpoint, double tolerance) {
      return (input >= (setpoint - tolerance)) && (input <= (setpoint + tolerance));
    }
  }

}
