
package frc.robot;

import com.revrobotics.CANSparkMax;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.swerveUtil.COTSFalconSwerveConstants;
import frc.lib.util.swerveUtil.RevSwerveModuleConstants;

public final class Constants {

    public static final class Swerve {

        // Spark Max Idle Modes
        public static final CANSparkMax.IdleMode driveIdleMode = CANSparkMax.IdleMode.kBrake;
        public static final CANSparkMax.IdleMode angleIdleMode = CANSparkMax.IdleMode.kBrake;

        // Max Output Powers
        public static final double drivePower = 1;
        public static final double anglePower = .9;

        // Gyro
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        // Swerve Module Type
        public static final COTSFalconSwerveConstants chosenModule =
            COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L1);
        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = chosenModule.canCoderInvert;
        public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;
        /* Motor Inverts */
        public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
        public static final double angleGearRatio = chosenModule.angleGearRatio;
        // the number of degrees that a single rotation of the turn motor turns the wheel.
        public static final double DegreesPerTurnRotation = 360/angleGearRatio;
        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;

        // encoder setup
        // meters per rotation
        public static final double wheelCircumference = chosenModule.wheelCircumference;
        public static final double driveRevToMeters =  wheelCircumference / (driveGearRatio );
        public static final double driveRpmToMetersPerSecond = driveRevToMeters/60 ;
        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(23.75);
        public static final double wheelBase = Units.inchesToMeters(23.75);
        /* Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));
        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 20;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;
        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;
        /* Angle Motor PID Values */
        public static final double angleKP = 0.05;
        public static final double angleKI = 0;
        public static final double angleKD = 0;
        public static final double angleKFF = 0;

        /* Drive Motor info  */
        public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;

        public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * wheelCircumference)
                / driveGearRatio;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.04;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKFF = 1 / kDriveWheelFreeSpeedRps;
        /** Meters per Second */
        public static final double maxSpeed = 3.6576;
        /** Radians per Second */
        public static final double maxAngularVelocity = 5.0;
        public static double angleRampRate = 0;

        /* CanCoder Constants */
        public static final CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();

        public static class Modules {
            /* Module Specific Constants */
        /* Front Left Module - Module 0 */
            public static final class Mod0 {

                public static final int driveMotorID = 2;
                public static final int angleMotorID = 3;
                public static final int canCoderID = 10;
                public static final Rotation2d angleOffset = Rotation2d.fromDegrees(162.146-180); //Rotation2d.fromDegrees(37.7);
                public static final RevSwerveModuleConstants constants =
                    new RevSwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
            }

            /* Front Right Module - Module 1 */
            public static final class Mod1 {
                public static final int driveMotorID = 4;
                public static final int angleMotorID = 5;
                public static final int canCoderID = 11;
                public static final Rotation2d angleOffset = Rotation2d.fromDegrees(26.015);
                public static final RevSwerveModuleConstants constants =
                    new RevSwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
            }

            /* Back Left Module - Module 2 */
            public static final class Mod2 {
                public static final int driveMotorID = 6;
                public static final int angleMotorID = 7;
                public static final int canCoderID = 12;
                public static final Rotation2d angleOffset = Rotation2d.fromDegrees(263.603);
                public static final RevSwerveModuleConstants constants =
                    new RevSwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
            }

            /* Back Right Module - Module 3 */
            public static final class Mod3 {
                public static final int driveMotorID = 8;
                public static final int angleMotorID = 9;
                public static final int canCoderID = 13;
                public static final Rotation2d angleOffset = Rotation2d.fromDegrees(317.021);
                public static final RevSwerveModuleConstants constants =
                    new RevSwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
            }
        }
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 2;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI * 16;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI * 16;

        public static final double X_kP = 5;
        public static final double X_kI = 0;
        public static final double X_kD = 0;

        public static final double Y_kP = 5;
        public static final double Y_kI = 0;
        public static final double Y_kD = 0;

        public static final double THETA_kP = 6.2;
        public static final double THETA_kI = 0;
        public static final double THETA_kD = 0;


        // Motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
                new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class NeoMotorConstants {
        public static final double kFreeSpeedRpm = 5676;
    }  

  public static class SHOOTER {
    public static final int shooterMotor_1 = 0;
    public static final int shooterMotor_2 = 0;
    public static final int shooterMotor_3 = 0;

  }

  public static class INTAKE {
    public static final int intakeMotor_1 = 0;
    public static final int intakeMotor_2 = 0;
  }

  public static class OIConstants {
    public static final int xboxController = 0;
    public static final int panelController = 0;
  }

  public static class VALUES {

    public static class INTAKE {
      
      public static final double GetNoteValue = 1;
      public static final double PidIntakeTolerance = 5;
      public static final double PidIntakeKP = 1;
      public static final double PidIntakeKI = 0;
      public static final double PidIntakeKD = 0;

    public static class SHOOTER {
  
      public static final double AngleMotorOpenLoopRampRate = 0.5;
      public static final double PidShooterAngleTolerance = 5;
      public static final double PidShooterAngleKP = 1;
      public static final double PidShooterAngleKI = 0;
      public static final double PidShooterAngleKD = 0;

      public static final double PidShooterShootKP = 1;
      public static final double PidShooterShootKI = 0;
      public static final double PidShooterShootKD = 0;
      public static final double PidShooterRPMTolerance = 5;
}
}

public static class POISTIONS {

  public static final double FeedingPositionIntake = 1;
  public static final double FeedingPositionShooter = 1;

}

public static class FEEDER {

  public static final double FeederForwardSpeed = 0.7;
  public static final double FeederBackwardSpeed = -0.7;

      }
  }

}
