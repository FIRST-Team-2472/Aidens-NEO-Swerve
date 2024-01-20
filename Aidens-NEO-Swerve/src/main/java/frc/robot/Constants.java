// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class ControllerConstants{
    // has our joystick and controller usb ports
    public static final int kLeftJoystickControllerPort = 0;
    public static final int kRightJoystickControllerPort = 1;
    public static final int kXboxControllerPort = 3;
  }
  public static final class ModuleConstants {
    //gear ratios wheel diameters and encoder numbers
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kDriveMotorGearRatio = 1/ 6.75; // look up the module gear ratio (confirmed)
    public static final double kTurningMotorGearRatio = 1 / 12.8; // confirmed for all mk4 modules
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
    public static final double kPTurning = 0.2;
  }

  public static final class SensorConstants{
    // extra sensors
    public static final int kPigeonID = 13;
  }

  public static final class OIConstants{
    //make this bigger  if it moves on its own lol
    public static final double kDeadband = 0.2;

  }
  public static final class DriveConstants{
    // configure our robot dimensions and motor controller/encoder ports
    public static final double kTrackWidth = Units.inchesToMeters(34.5);
    //distance between right and left wheels
    public static final double kWheelBase = Units.inchesToMeters(34.5);
    //distance between front and back wheels
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, -kTrackWidth/ 2),
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2)
    );

    //motor and encoder ports IMPORTANT

    //Front Left
    public static final int kFrontLeftDriveMotorPort = 55;
    public static final int kFrontLeftTurningMotorPort = 54; // fill these out then delete this comment
    public static final boolean kFrontLeftDriveEncoderReversed = true;
    public static final boolean kFrontLeftTurningEncoderReversed = true;
    public static final int kFrontLeftDriveAbsoluteEncoderPort = 61;
    public static final double kFrontLeftDriveAbsoluteEncoderOffsetAng = 357.4;
    public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;

    //Front Right
    public static final int kFrontRightDriveMotorPort = 53;
    public static final int kFrontRightTurningMotorPort = 51;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kFrontRightTurningEncoderReversed = true;
    public static final int kFrontRightDriveAbsoluteEncoderPort = 62;
    public static final double kFrontRightDriveAbsoluteEncoderOffsetAng = 255.41;
    public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
 
    //Back Left
    public static final int kBackLeftDriveMotorPort = 43;
    public static final int kBackLeftTurningMotorPort = 50;
    public static final boolean kBackLeftDriveEncoderReversed = true;
    public static final boolean kBackLeftTurningEncoderReversed = true;
    public static final int kBackLeftDriveAbsoluteEncoderPort = 60;
    public static final double kBackLeftDriveAbsoluteEncoderOffsetAng = 284.5;
    public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;

    //Back Right
    public static final int kBackRightDriveMotorPort = 56;
    public static final int kBackRightTurningMotorPort = 57;
    public static final boolean kBackRightDriveEncoderReversed = false;
    public static final boolean kBackRightTurningEncoderReversed = true;
    public static final int kBackRightDriveAbsoluteEncoderPort = 59;
    public static final double kBackRightDriveAbsoluteEncoderOffsetAng = 218.8;
    public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

    // run at full speed to test these then delete this comment
    public static final double kPhysicalMaxSpeedMetersPerSecond = 4;
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 7 * 2 * Math.PI;

    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 1.2;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
                kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 8;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = Math.PI * 2;
  }

  public static final class AutoConstants{
    public static final double kMaxSpeedMetersPerSecond = 6.8;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1.0;
    public static final double kPXController = 1.5;
    public static final double kPYController = 1.5;
    public static final double kPThetaController = 1.5;
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
          kMaxSpeedMetersPerSecond, 
        kMaxAccelerationMetersPerSecondSquared);
    public static double kThetaController;
  }
}
