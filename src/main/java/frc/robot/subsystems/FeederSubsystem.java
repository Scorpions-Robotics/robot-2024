package frc.robot.subsystems;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class FeederSubsystem extends SubsystemBase {
  
  public WPI_VictorSPX feedmotor = new WPI_VictorSPX(7);
  public DigitalInput detector = new DigitalInput(9);
  public boolean magstatus;

  public FeederSubsystem() {}

  public void forward(){
    feedmotor.set(0.8);
  }

  public void backward(){
    feedmotor.set(-0.8);
  }

  public boolean ismagfilled(){
    if (detector.get()){
      return false;
    }
    else{
      return true;
    }
  }



  public void stop(){
    feedmotor.set(0);
  }

  @Override
  public void periodic() {

    SmartDashboard.putBoolean("bakacaz", ismagfilled());

  }
}
