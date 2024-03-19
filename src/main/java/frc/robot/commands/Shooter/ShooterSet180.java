package frc.robot.commands.Shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterSet180 extends PIDCommand {

  public ShooterSet180(ShooterSubsystem m_shooter) {
    super(

       // new PIDController(0.04,
        new PIDController(0.035,
            0.003,
            Constants.values.shooter.PidShooterAngleKD),
        () -> m_shooter.getMappedOutput(),
        () -> 180,
        output -> {

          m_shooter.ShooterAngleMotorOutput(output * -.13);

        });
    addRequirements(m_shooter);
    getController().setTolerance(Constants.values.shooter.PidShooterAngleTolerance);

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
