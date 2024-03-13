package frc.robot.commands.Shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.NetworkSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class VisionShooter extends PIDCommand {

  public VisionShooter(ShooterSubsystem m_shooter, NetworkSubsystem m_network , DoubleSupplier angle) {
    super(

        new PIDController(0.04,
            0.03,
            Constants.values.shooter.PidShooterAngleKD),
        () -> m_network.getY(),
        () -> 0,
        output -> {

          m_shooter.ShooterAngleMotorOutput(output * -.13);

        });
    addRequirements(m_shooter);
    getController().setTolerance(Constants.values.shooter.PidShooterAngleTolerance);
      SmartDashboard.putNumber("pid shooter gonderilen pozisyon", angle.getAsDouble());

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
