package frc.robot;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.Constants.Swerve;
import frc.robot.commands.Swerve.TeleopDrive;
import frc.robot.subsystems.Swerve.SwerveBase;

public class RobotContainer {

  XboxController xboxController = 
  new XboxController(Constants.OIConstants.xboxController);

  XboxController panelController = 
  new XboxController(Constants.OIConstants.panelController);

  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  private final JoystickButton zeroGyro = new JoystickButton(xboxController, XboxController.Button.kA.value);
  private final JoystickButton autoMove = new JoystickButton(xboxController, XboxController.Button.kB.value);


  SwerveBase m_SwerveBase = new SwerveBase();  

  public RobotContainer() {

    m_SwerveBase.setDefaultCommand(
      new TeleopDrive(
          m_SwerveBase,
          () -> -xboxController.getRawAxis(translationAxis),
          () -> -xboxController.getRawAxis(strafeAxis),
          () -> -xboxController.getRawAxis(rotationAxis),
          () -> xboxController.getRawButtonPressed(XboxController.Button.kY.value),
          () -> false
      )
  );

    configureBindings();
    
  }


  private void configureBindings() {

    zeroGyro.onTrue(new InstantCommand(() -> m_SwerveBase.zeroGyro()));

  }


  public Command getAutonomousCommand() {
  return null;
  }
}
