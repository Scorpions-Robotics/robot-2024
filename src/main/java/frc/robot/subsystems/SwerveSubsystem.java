package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.path.PathPlannerTrajectory;
// import com.pathplanner.lib.commands.PP
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.I2C;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class SwerveSubsystem extends SubsystemBase {

    private final SwerveModule frontLeft = new SwerveModule(
        DriveConstants.kFrontLeftDriveMotorPort, 
        DriveConstants.kFrontLeftTurnMotorPort, 
        DriveConstants.kFrontLeftAbsoluteEncoderReversed, 
        DriveConstants.kFrontLeftAbsoluteEncoderOffset, 
        DriveConstants.kFrontLeftDriveMotorReversed, 
        DriveConstants.kFrontLeftTurnMotorReversed, 
        DriveConstants.kFrontLeftAbsoluteEncoderPort
    );
    
    private final SwerveModule frontRight = new SwerveModule(
        DriveConstants.kFrontRightDriveMotorPort, 
        DriveConstants.kFrontRightTurnMotorPort, 
        DriveConstants.kFrontRightAbsoluteEncoderReversed, 
        DriveConstants.kFrontRightAbsoluteEncoderOffset, 
        DriveConstants.kFrontRightDriveMotorReversed, 
        DriveConstants.kFrontRightTurnMotorReversed, 
        DriveConstants.kFrontRightAbsoluteEncoderPort
    );

    private final SwerveModule backLeft = new SwerveModule(
        DriveConstants.kBackLeftDriveMotorPort, 
        DriveConstants.kBackLeftTurnMotorPort, 
        DriveConstants.kBackLeftAbsoluteEncoderReversed, 
        DriveConstants.kBackLeftAbsoluteEncoderOffset, 
        DriveConstants.kBackLeftDriveMotorReversed, 
        DriveConstants.kBackLeftTurnMotorReversed, 
        DriveConstants.kBackLeftAbsoluteEncoderPort
    );

    private final SwerveModule backRight = new SwerveModule(
        DriveConstants.kBackRightDriveMotorPort, 
        DriveConstants.kBackRightTurnMotorPort, 
        DriveConstants.kBackRightAbsoluteEncoderReversed, 
        DriveConstants.kBackRightAbsoluteEncoderOffset, 
        DriveConstants.kBackRightDriveMotorReversed, 
        DriveConstants.kBackRightTurnMotorReversed, 
        DriveConstants.kBackRightAbsoluteEncoderPort
    );

    private final AHRS navX = new AHRS(SerialPort.Port.kMXP);

    private SwerveDriveOdometry kSwerveDriveOdometry = new SwerveDriveOdometry(Constants.DriveConstants.kDriveKinematics, new Rotation2d(), getModulePositions());;

    public SwerveSubsystem() {
        // navX.calibrate();
        new Thread(() -> {
            try {
                while(navX.isCalibrating()) {

                }

                zeroHeading();
            } catch (Exception e) {

            }
        }).start();
    }

    public void zeroHeading() {
        navX.zeroYaw();
    }
    
    /*public void setAngle(double angle) {
        navX.setAngleAdjustment(angle);
    }*/

    public double getHeading() {
        return navX.getYaw() + 180;
    }

    public double getInclination() {
        return navX.getPitch();
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose2d() {
        return kSwerveDriveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        kSwerveDriveOdometry.resetPosition(getRotation2d(), getModulePositions(), pose);
    }
 
    // public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
    //     return new SequentialCommandGroup(
    //          new InstantCommand(() -> {
    //            // Reset odometry for the first path you run during auto
    //            if(isFirstPath){
    //                this.resetOdometry(traj.getInitialHolonomicPose());
    //            }
    //          }),
    //          new PPSwerveControllerCommand(
    //              traj, 
    //              this::getPose2d, // Pose supplier
    //              DriveConstants.kDriveKinematics, // SwerveDriveKinematics
    //              new PIDController(DriveConstants.kPX, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
    //              new PIDController(DriveConstants.kPY, 0, 0), // Y controller (usually the same values as X controller)
    //              new PIDController(DriveConstants.kPTheta, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
    //              this::setModuleStatesAuton, // Module states consumer
    //              false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
    //              this // Requires this drive subsystem
    //          )
    //      );
    //  }

    @Override
    public void periodic() {
        kSwerveDriveOdometry.update(getRotation2d(), getModulePositions());

        SmartDashboard.putNumber("frontLeft", frontLeft   .getTurnPosition());
        SmartDashboard.putNumber("frontRight", frontRight  .getTurnPosition());
        SmartDashboard.putNumber("backLeft", backLeft    .getTurnPosition());
        SmartDashboard.putNumber("backRight", backRight   .getTurnPosition());
        SmartDashboard.putNumber("backLeft Turn Current", backLeft.getTurnCurrent());
        SmartDashboard.putNumber("battery voltage", RobotController.getBatteryVoltage());
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setDriveMotorsIdleMode(IdleMode idleMode) {
        frontLeft.setDriveMotorIdleMode(idleMode);
        frontRight.setDriveMotorIdleMode(idleMode);
        backLeft.setDriveMotorIdleMode(idleMode);
        backRight.setDriveMotorIdleMode(idleMode);
    }
    
    public void setModulesStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }    

    public void setModuleStatesAuton(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

        frontLeft.setDesiredStateAuton(desiredStates[0]);
        frontRight.setDesiredStateAuton(desiredStates[1]);
        backLeft.setDesiredStateAuton(desiredStates[2]);
        backRight.setDesiredStateAuton(desiredStates[3]);
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            new SwerveModulePosition(frontLeft.getDrivePosition(), new Rotation2d(frontLeft.getTurnPosition())),
            new SwerveModulePosition(frontRight.getDrivePosition(), new Rotation2d(frontRight.getTurnPosition())),
            new SwerveModulePosition(backLeft.getDrivePosition(), new Rotation2d(backLeft.getTurnPosition())),
            new SwerveModulePosition(backRight.getDrivePosition(), new Rotation2d(backRight.getTurnPosition()))
        };
    }

    public void setOdometryPosition(Pose2d startLocation) {
        kSwerveDriveOdometry.resetPosition(getRotation2d(), getModulePositions(), startLocation);
    }

    public void resetEncoders() {
        backLeft.resetEncoders();
        backRight.resetEncoders();
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
    }
}