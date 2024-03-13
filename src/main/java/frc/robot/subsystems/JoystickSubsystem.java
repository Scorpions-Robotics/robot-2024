
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class JoystickSubsystem extends SubsystemBase {

  public int intakemod;
  public int shootermod;

  public double intakedeger;
  public double shooterdeger;

  public JoystickSubsystem() {


  }
  public void shootermodchange(int mod){
    shootermod = mod;
  }

  public void intakemodchange(int mod){
    intakemod = mod;
  }

public double getintakevalue(){
return intakedeger;
}

public double getshootervalue(){
  return shooterdeger;
  }


  @Override
  public void periodic() {


switch (intakemod){
case 0:
intakedeger = 1.6;
break;

case 1:
intakedeger = 12.5;
break;
}



switch (shootermod){
  case 0:
  shooterdeger = -2.0;
  break;
  
  case 1:
  shooterdeger = 70;
  break;

  case 2:
  shooterdeger = 164;
  break;
  }
  


  SmartDashboard.putNumber("intakemod", intakemod);
  SmartDashboard.putNumber("shootermod", shootermod);
  SmartDashboard.putNumber("intakedeger", intakedeger);
  SmartDashboard.putNumber("shooterdeger", shooterdeger);

  }
}
