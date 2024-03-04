package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
// intake nota motorları
public static CANSparkMax intakeMotor1 = new CANSparkMax(0, MotorType.kBrushless);
//intake main motor
public static CANSparkMax intakeMotor = new CANSparkMax(0, MotorType.kBrushless);
public RelativeEncoder IntakeEncoder;

  public IntakeSubsystem() {
    intakeMotor.setIdleMode(IdleMode.kBrake);
    IntakeEncoder = intakeMotor.getEncoder();
  }

  public void brakemode(){
    intakeMotor.setIdleMode(IdleMode.kBrake);

  }
  public void coastmode(){
    intakeMotor.setIdleMode(IdleMode.kCoast);
  }

  public double getRawEncoderOutput(){
    return IntakeEncoder.getPosition();
  }
  public double getMappedOutput(){
    return IntakeEncoder.getPosition() * -10;
  }
  public void reset() {
    IntakeEncoder.setPosition(0);
  }
  
    public void NewIntakeMotorOutput(double value){
      intakeMotor.set(value);
    }

    public void getNote(){
      intakeMotor1.set(Constants.values.intake.GetNoteValue);
    }

    public void StopNoteMotor(){
      intakeMotor1.set(0);
    }

  
  @Override
  public void periodic() {
   
  }
}
