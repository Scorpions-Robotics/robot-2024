package frc.robot.subsystems;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class FeederSubsystem extends SubsystemBase {
  
  public static CANSparkMax feederMotor = new CANSparkMax(0, MotorType.kBrushless);

  public FeederSubsystem() {}

  public void forward(){
    feederMotor.set(Constants.values.feeder.FeederForwardSpeed);
  }

  public void backward(){
    feederMotor.set(Constants.values.feeder.FeederBackwardSpeed);
  }

  public void stop(){
    feederMotor.set(0);
  }

  @Override
  public void periodic() {
  }
}
