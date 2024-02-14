package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

public static CANSparkMax ShooterAngleMotor = new CANSparkMax(Constants.ports.shooter_motor_1, MotorType.kBrushless);
public static CANSparkMax ShooterThrowMotor1 = new CANSparkMax(Constants.ports.shooter_motor_2, MotorType.kBrushless);
public static CANSparkMax ShooterThrowMotor2 = new CANSparkMax(Constants.ports.shooter_motor_3, MotorType.kBrushless);


public RelativeEncoder ShooterThrow1Encoder;
public RelativeEncoder ShooterThrow2Encoder;
public RelativeEncoder ShooterAngleEncoder;

  public ShooterSubsystem() {
  
    ShooterAngleMotor.setIdleMode(IdleMode.kBrake);
    ShooterAngleEncoder = ShooterAngleMotor.getEncoder();
    ShooterThrow1Encoder = ShooterThrowMotor1.getEncoder();
    ShooterThrow2Encoder = ShooterThrowMotor2.getEncoder();
    ShooterAngleMotor.setOpenLoopRampRate(Constants.values.shooter.AngleMotorOpenLoopRampRate);
  }


  public void brakemode1(){
    ShooterThrowMotor1.setIdleMode(IdleMode.kBrake);

  }
  public void brakemode2(){
    ShooterThrowMotor2.setIdleMode(IdleMode.kBrake);

  }
  public void coastmode1(){
    ShooterThrowMotor1.setIdleMode(IdleMode.kCoast);
  }
  public void coastmode2(){
    ShooterThrowMotor1.setIdleMode(IdleMode.kCoast);
  }
  public void barkeall(){
    ShooterThrowMotor1.setIdleMode(IdleMode.kBrake);
    ShooterThrowMotor2.setIdleMode(IdleMode.kBrake);
  }
  public void coastall(){
    ShooterThrowMotor1.setIdleMode(IdleMode.kCoast);
    ShooterThrowMotor2.setIdleMode(IdleMode.kCoast);
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
  
  public double getRpmOutput1(){
    return ShooterThrow1Encoder.getVelocity() * 600 / 4096.0;
  }
  public double getRpmOutput2(){
    return ShooterThrow2Encoder.getVelocity() * 600 / 4096.0;
  }

    public void ShooterThrow1MotorOutput(double value){
      ShooterThrowMotor1.set(value);
    }
    public void ShooterThrow2MotorOutput(double value){
      ShooterThrowMotor2.set(value);
    }

    public void ShooterAngleMotorOutput(double value){
      ShooterAngleMotor.set(value);
    }
    public void ShooterThrow1MotorStop(){
      ShooterThrowMotor1.set(0);
    }
    public void ShooterThrow2MotorStop(){
      ShooterThrowMotor2.set(0);
    }
    public void ShooterThrowAllMotorStop(){
      ShooterThrowMotor1.set(0);
      ShooterThrowMotor2.set(0);
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
