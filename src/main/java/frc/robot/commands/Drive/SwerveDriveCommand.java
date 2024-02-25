package frc.robot.commands.Drive;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;


public class SwerveDriveCommand extends Command {

    private final SwerveSubsystem driveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Boolean> fieldOrientationFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    public SwerveDriveCommand(SwerveSubsystem driveSubsystem, Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction, Supplier<Boolean> fieldOrientatedFunction) {
        this.driveSubsystem = driveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.fieldOrientationFunction = fieldOrientatedFunction;
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("Field Oriented", fieldOrientationFunction.get());
        double xSpeed = (DriveConstants.slowMode) ? xSpdFunction.get() : xSpdFunction.get();
        double ySpeed = (DriveConstants.slowMode) ? ySpdFunction.get() : ySpdFunction.get();
        double turningSpeed = (DriveConstants.slowMode) ? turningSpdFunction.get() / 5 : turningSpdFunction.get();

        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        ChassisSpeeds chassisSpeeds;
        if (fieldOrientationFunction.get()) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, driveSubsystem.getRotation2d());
        }
        else {
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        
        driveSubsystem.setModulesStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}