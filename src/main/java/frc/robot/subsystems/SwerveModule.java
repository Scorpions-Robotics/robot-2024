
package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.DriveConstants;

public class SwerveModule extends SubsystemBase {

  CANSparkMax driveMotor;
  CANSparkMax turningMotor;

  RelativeEncoder driveEncoder;
  RelativeEncoder turningEncoder;

  PIDController turningPidController;

  CANcoder absoluteEncoder;
  boolean absoluteEncoderReversed;
  double absoluteEncoderOffsetRad;

  int driveMotorId;

  public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
      int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

    this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
    this.driveMotorId = driveMotorId;
    this.absoluteEncoderReversed = absoluteEncoderReversed;

    absoluteEncoder = new CANcoder(absoluteEncoderId);

    driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
    turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

    driveMotor.setInverted(driveMotorReversed);
    turningMotor.setInverted(turningMotorReversed);

    driveEncoder = driveMotor.getEncoder();
    turningEncoder = turningMotor.getEncoder();

    driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
    driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
    turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
    turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

    turningPidController = new PIDController(0.5, 0, 0);
    turningPidController.enableContinuousInput(-Math.PI, Math.PI);
    driveMotor.setOpenLoopRampRate(0.4);
    turningMotor.setOpenLoopRampRate(0.4);

  }

  public double getDrivePosition() {
    SmartDashboard.putNumber("swervePos"+ driveMotorId, driveEncoder.getPosition());
    return driveEncoder.getPosition();
  }

  public SwerveModulePosition getPosition(){

    return new SwerveModulePosition(
        driveEncoder.getPosition() ,
        new Rotation2d(turningEncoder.getPosition()));
    
  }

  public double getTurningPosition() {
    SmartDashboard.putNumber("swervePosTur"+ driveMotorId, turningEncoder.getPosition());
    return  turningEncoder.getPosition();
  }

   public double getTurningPosition2() {
    return  turningEncoder.getPosition() * 50;
  }

  public double getDriveVelocity() {
    SmartDashboard.putNumber("swerveVelo"+ driveMotorId, driveEncoder.getVelocity());

    return driveEncoder.getVelocity();
  }

  public double getTurningVelocity() {
    return turningEncoder.getVelocity();
  }

  public double getAbsoluteEncoderRad() {
    StatusSignal<Double> absoluteencoderradian = absoluteEncoder.getAbsolutePosition();
    double angle = absoluteencoderradian.getValueAsDouble();
    angle *= 2.0 * Math.PI;
    angle -= absoluteEncoderOffsetRad;
    return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
  }

  public void resetEncoders() {
    driveEncoder.setPosition(0);
    turningEncoder.setPosition(getAbsoluteEncoderRad());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
  }

  public void stop() {
    driveMotor.set(0);
    turningMotor.set(0);
  }

  public void setDesiredState(SwerveModuleState state) {
    if (Math.abs(state.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }
    state = SwerveModuleState.optimize(state, getState().angle);
    driveMotor.set(4 * (state.speedMetersPerSecond) / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    turningMotor.set(0.5*(turningPidController.calculate(getTurningPosition(), state.angle.getRadians())));
    SmartDashboard.putString("Swerve[" + absoluteEncoder.getDeviceID() + "] state", state.toString());
    SmartDashboard.putNumber("status",
        (state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond) * 3);
    SmartDashboard.putNumber("digeri", turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
  }

  public void OtosetDesiredState(SwerveModuleState state) {
    if (Math.abs(state.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }
    state = SwerveModuleState.optimize(state, getState().angle);
    driveMotor.set((state.speedMetersPerSecond) / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    turningMotor.set((turningPidController.calculate(getTurningPosition(), state.angle.getRadians())));
    SmartDashboard.putString("Swerve[" + absoluteEncoder.getDeviceID() + "] state", state.toString());
    SmartDashboard.putNumber("OHOOHOHOHOHOHOOHOHOOHOHOOHO",
        (state.speedMetersPerSecond) / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    SmartDashboard.putNumber("ASDSKMA SKDASDMASPKDASDAPKMP", (turningPidController.calculate(getTurningPosition(), state.angle.getRadians())));
  }

  @Override
  public void periodic() {
        SmartDashboard.putString("POSSSSS"+ driveMotorId, getPosition().toString());
  }
}
