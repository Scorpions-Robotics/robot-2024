package frc.robot.commands.Shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

public class PidSetRpm extends PIDCommand {

  public PidSetRpm(ShooterSubsystem m_shooter, double estimatedRpm) {
    super(
        new PIDController(Constants.values.shooter.PidShooterShootKP,
            Constants.values.shooter.PidShooterShootKI,
            Constants.values.shooter.PidShooterShootKD),

        () -> m_shooter.getRpmOutput1(),

        () -> estimatedRpm,

        output -> {

          if (estimatedRpm > m_shooter.getRpmOutput1()) {
            m_shooter.ShooterThrow1MotorOutput(output);
            m_shooter.ShooterThrow2MotorOutput(output);

          }

        });

    getController().setTolerance(Constants.values.shooter.PidShooterRPMTolerance);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
