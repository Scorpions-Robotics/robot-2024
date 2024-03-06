package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.values.shooter;

public class ShooterSubsystem extends SubsystemBase {

public static CANSparkMax ShooterAngleMotor = new CANSparkMax(Constants.ports.shooter_motor_3, MotorType.kBrushless);
public static CANSparkMax ShooterThrowMotor1 = new CANSparkMax(Constants.ports.shooter_motor_1, MotorType.kBrushless);
public static CANSparkMax ShooterThrowMotor2 = new CANSparkMax(Constants.ports.shooter_motor_2, MotorType.kBrushless);

public RelativeEncoder ShooterThrow1Encoder;
public RelativeEncoder ShooterThrow2Encoder;
public RelativeEncoder ShooterAngleEncoder;

DigitalInput shooterswitch = new DigitalInput(8);

public void setencodervalue(double value){
  ShooterAngleEncoder.setPosition(value);
}

  public ShooterSubsystem() {
    ShooterAngleMotor.setIdleMode(IdleMode.kBrake);
    ShooterAngleEncoder = ShooterAngleMotor.getEncoder();
    ShooterThrow1Encoder = ShooterThrowMotor1.getEncoder();
    ShooterThrow2Encoder = ShooterThrowMotor2.getEncoder();
    ShooterAngleMotor.setOpenLoopRampRate(Constants.values.shooter.AngleMotorOpenLoopRampRate);
    setencodervalue(0.525);


  }

  public boolean getshooterswitchvalue(){
    return shooterswitch.get();
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
    return ShooterThrow1Encoder.getVelocity();
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

    public void ShooterThrowMotorOutput(double value){
      ShooterThrowMotor1.set(value);
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
    public void ShootertoFeederPos(){
      if(!shooterswitch.get()){
        ShooterAngleMotor.set(0.1);
      }else{
        ShooterAngleMotor.stopMotor();
      }
    }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("shooter rpm 1",getRpmOutput1());
    SmartDashboard.putNumber("shooter rpm 2",getRpmOutput2());
    SmartDashboard.putNumber("Shooter degree", getMappedOutput());


if(getshooterswitchvalue()){
  ShooterAngleEncoder.setPosition(0);
  }
SmartDashboard.putBoolean("deger", getshooterswitchvalue());

}

}
