package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  // intake nota motorları
  public static CANSparkMax intakeMotor1 = new CANSparkMax(12,MotorType.kBrushed);
  // intake main motor
  public static CANSparkMax intakeMotor = new CANSparkMax(Constants.ports.intake_motor_angle, MotorType.kBrushless);

  public RelativeEncoder IntakeEncoder;
  public DigitalInput intakedetector1 = new DigitalInput(8);
  public DigitalInput intakedetector2 = new DigitalInput(9);
  public boolean tekcalisma = true;
  public boolean digeride = false;

  public IntakeSubsystem() {
    intakeMotor.setIdleMode(IdleMode.kBrake);
    IntakeEncoder = intakeMotor.getEncoder();
  }

  public void digeridetrue(){
    digeride = true;

  }

  public void digeridefalse(){
    digeride = false;

  }

  public boolean digeridedondur(){
  return digeride;
  }

  public void brakemode() {
    intakeMotor.setIdleMode(IdleMode.kBrake);

  }

  public void coastmode() {
    intakeMotor.setIdleMode(IdleMode.kCoast);
  }

  public double getRawEncoderOutput() {
    return IntakeEncoder.getPosition();
  }

  public double getMappedOutput() {
    return IntakeEncoder.getPosition() * -10;
  }

  public void reset() {
    IntakeEncoder.setPosition(0);
  }

  public void NewIntakeMotorOutput(double value) {
    intakeMotor.set(value);
  }

  public void getNote() {
    intakeMotor1.set(0.67);
  }
  public void otogetNote() {
    intakeMotor1.set(0.7);
  }


  public void pushNote() {
    intakeMotor1.set(-1);
  }

  public void degistir() {
    tekcalisma = true;
  }

  public void degistirmee() {
    tekcalisma = true;
  }

  public void runpickupmotorswitch(double a) {


    if (tekcalisma) {
      while ((intakedetector1.get() && intakedetector2.get())) {
        intakeMotor1.set(a);
      }

      intakeMotor1.set(0);
    }

  }

  public void runpickupmotor(double a) {
    intakeMotor1.set(a);
  }

  public void StopNoteMotor() {
    intakeMotor1.set(0);
  }

  public void StopAngleMotor() {
    intakeMotor.set(0);
  }

  public Boolean intakeswitch1() {
    return intakedetector1.get();
  }

  public Boolean intakeswitch2() {
    return intakedetector2.get();
  }

  public Boolean intakeTwoStich(){
    if(!intakedetector2.get()|| !intakedetector1.get()){
      return true;
    }
    else{
      return false;
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("intake", getRawEncoderOutput());
    SmartDashboard.putBoolean("intake switch1", intakeswitch1());
    SmartDashboard.putBoolean("intake switch2", intakeswitch2());

    SmartDashboard.putBoolean("degistirme", tekcalisma);
  }
}