
package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {
  // bura acil konf
  public static class ModuleConstants {

    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kDriveMotorGearRatio = 1 / 6.75;
    public static final double kTurningMotorGearRatio = 0.0453;
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
    public static final double kPTurning = 0.5;
  }

  // bura acil konf
  public static final class DriveConstants {

    public static final double kTrackWidth = Units.inchesToMeters(23);
    // Distance between right and left wheels
    public static final double kWheelBase = Units.inchesToMeters(23);
    // Distance between front and back wheels
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

    // burası tamam
    public static final int kFrontLeftDriveMotorPort = 41;
    public static final int kBackLeftDriveMotorPort = 3;
    public static final int kFrontRightDriveMotorPort = 10;
    public static final int kBackRightDriveMotorPort = 11;

    public static final int kFrontLeftTurningMotorPort = 2;
    public static final int kBackLeftTurningMotorPort = 1;
    public static final int kFrontRightTurningMotorPort = 8;
    public static final int kBackRightTurningMotorPort = 9;

    // buraya kadar

    public static final boolean kFrontLeftTurningEncoderReversed = false;
    public static final boolean kBackLeftTurningEncoderReversed = false;
    public static final boolean kFrontRightTurningEncoderReversed = false;
    public static final boolean kBackRightTurningEncoderReversed = false;

    public static final boolean kFrontLeftDriveEncoderReversed = true;
    public static final boolean kBackLeftDriveEncoderReversed = true;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kBackRightDriveEncoderReversed = false;

    public static final int kFrontLeftDriveAbsoluteEncoderPort = 2;
    public static final int kBackLeftDriveAbsoluteEncoderPort = 4;
    public static final int kFrontRightDriveAbsoluteEncoderPort = 3;
    public static final int kBackRightDriveAbsoluteEncoderPort = 1;

    public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = true;
    public static final boolean kBackLeftDriveAbsoluteEncoderReversed = true;
    public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
    public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

    public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = -0.4101;
    public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = -0.1276;
    public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = -0.216064;
    public static final double kBackRightDriveAbsoluteEncoderOffsetRad = -0.126221;

    public static final double kPhysicalMaxSpeedMetersPerSecond = 3;
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 4 * 2 * Math.PI;

    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
        kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 10;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 10;
  }

  // bura acil
  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kDriverYAxis = 1;
    public static final int kDriverXAxis = 0;
    public static final int kDriverRotAxis = 4;
    public static final int kDriverFieldOrientedButtonIdx = 1;

    public static final double kDeadband = 0.05;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
    public static final double kMaxAngularSpeedRadiansPerSecond = //
        DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
    public static final double kPXController = 1.5;
    public static final double kPYController = 1.5;
    public static final double kPThetaController = 3;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond,
            kMaxAngularAccelerationRadiansPerSecondSquared);
  }

  public static class ports {
    public static final int intake_motor_note = 0;
    public static final int intake_motor_angle = 14;
    public static final int shooter_motor_1 = 6;
    public static final int shooter_motor_2 = 5;
    public static final int shooter_motor_3 = 15;

  }

  public static class controller {
    public static final int controller = 1;
  }

  /*
   * ___ __ _ _ _ _____ _ _
   * / __|___ _ _ / _(_)__ _ _ _ _ _ __ _| |_(_)___ _ _ |_ _|_ _| |__| |___
   * | (__/ _ \ ' \| _| / _` | || | '_/ _` | _| / _ \ ' \ | |/ _` | '_ \ / -_)
   * \___\___/_||_|_| |_\__, |\_,_|_| \__,_|\__|_\___/_||_| |_|\__,_|_.__/_\___|
   * |___/
   * 
   */

  public static class values {

    public static class intake {
      public static final double GetNoteValue = 1;
      public static final double PidIntakeTolerance = 1;
      public static final double PidIntakeKP = 1;
      public static final double PidIntakeKI = 0;
      public static final double PidIntakeKD = 0;
    }

    public static class shooter {
      public static final double AngleMotorOpenLoopRampRate = 0.5;
      public static final double PidShooterAngleTolerance = 0.1;
      public static final double PidShooterAngleKP = 0.05;
      public static final double PidShooterAngleKI = 0;
      public static final double PidShooterAngleKD = 0;

      public static final double PidShooterShootKP = 1;
      public static final double PidShooterShootKI = 0;
      public static final double PidShooterShootKD = 0;
      public static final double PidShooterRPMTolerance = 25;

    }

    public static class positions {

      public static final double FeedingPositionIntake = 1;
      public static final double FeedingPositionShooter = 1;

    }

    public static class feeder {

      public static final double FeederForwardSpeed = 0.7;
      public static final double FeederBackwardSpeed = -0.7;

    }
  }
}
