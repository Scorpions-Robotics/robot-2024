package frc.robot;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.controller;

public class RobotContainer {
  XboxController controller = new XboxController(Constants.controller.controller);
  

  public RobotContainer() {

    configureBindings();
    
  }


  private void configureBindings() {

  }


  public Command getAutonomousCommand() {
  return null;
  }
}
