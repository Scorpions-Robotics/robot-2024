package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class FeederSubsystem extends SubsystemBase {

  public WPI_VictorSPX feedmotor = new WPI_VictorSPX(7);
  public DigitalInput detector = new DigitalInput(6);
  public boolean magstatus;
  public boolean count;
  IntakeSubsystem m_intake;
  boolean tekcalisma ;
  //public AnalogInput intechedetector = new AnalogInput(0);
  


  public FeederSubsystem() {

  }

  public void forward() {
    feedmotor.set(0.8);
  }

  public void backward() {
    feedmotor.set(-0.8);
  }
  
  public void runtillswitch(){
while((detector.get())){
  feedmotor.set(-0.5);


    while ((detector.get())) {
      feedmotor.set(-0.8);

    }
    feedmotor.set(0);

}

   }  
  
  //while((detector.getValue() > 250)&&(intechedetector.getValue() < 2000)){
  //feedmotor.set(-0.8);

  // }
  // feedmotor.set(0);

  // }

  // public boolean ismagfilled(){
  // if (detector.getValue() >){
  // return false;
  // }
  // else{
  // return true;
  // }
  // }

  public void stop() {
    feedmotor.set(0);
  }

  @Override
  public void periodic() {

    SmartDashboard.putBoolean("bakacaz", detector.get());

  }
}
