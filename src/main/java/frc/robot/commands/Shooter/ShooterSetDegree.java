package frc.robot.commands.Shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterSetDegree extends PIDCommand {

  public ShooterSetDegree(ShooterSubsystem m_shooter, Double angle) {
    super(

    new PIDController(Constants.values.shooter.PidShooterAngleKP,
         Constants.values.shooter.PidShooterAngleKI,
          Constants.values.shooter.PidShooterAngleKD),
        () -> m_shooter.getMappedOutput(),
        () -> angle,
        output -> {

          if (angle > m_shooter.getMappedOutput()) {
            m_shooter.ShooterAngleMotorOutput(-output);
          } 
          
          else if (angle < m_shooter.getMappedOutput()) {
            m_shooter.ShooterAngleMotorOutput(output);
          }


        });
        addRequirements(m_shooter);
        getController().setTolerance(Constants.values.shooter.PidShooterAngleTolerance);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
