package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

public static CANSparkMax ShooterThrowMotor = new CANSparkMax(Constants.ports.shooter_motor_2, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
public static CANSparkMax ShooterAngleMotor = new CANSparkMax(Constants.ports.shooter_motor_1, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);

public RelativeEncoder ShooterThrowEncoder;
public RelativeEncoder ShooterAngleEncoder;

  public ShooterSubsystem() {
  
    ShooterAngleMotor.setIdleMode(IdleMode.kBrake);
    ShooterAngleEncoder = ShooterAngleMotor.getEncoder();
     ShooterThrowEncoder = ShooterThrowMotor.getEncoder();
    ShooterAngleMotor.setOpenLoopRampRate(Constants.values.shooter.AngleMotorOpenLoopRampRate);
  }


  public void brakemode(){
    ShooterThrowMotor.setIdleMode(IdleMode.kBrake);

  }
  public void coastmode(){
    ShooterThrowMotor.setIdleMode(IdleMode.kCoast);
  }

  public double getRawEncoderOutput(){
    return ShooterAngleEncoder.getPosition();
  }
  public double getMappedOutput(){
    return ShooterAngleEncoder.getPosition() * -10;
  }
  public void AngleEncoderReset() {
    ShooterAngleEncoder.setPosition(0);
  }
  
  public double getRpmOutput(){
    return ShooterThrowEncoder.getVelocity() * 600 / 4096.0;
  }

    public void ShooterThrowMotorOutput(double value){
      ShooterThrowMotor.set(value);
    }

    public void ShooterAngleMotorOutput(double value){
      ShooterAngleMotor.set(value);
    }


    public void ShooterThrowMotorStop(){
      ShooterThrowMotor.set(0);
    }

    public void ShooterAngleMotorStop(){
      ShooterAngleMotor.set(0);
    }


    public double calculateShootingRpm(double measure){
      //bugunluk bu kadar yeter.
      return 0;

    }

  @Override
  public void periodic() {

  }
}
