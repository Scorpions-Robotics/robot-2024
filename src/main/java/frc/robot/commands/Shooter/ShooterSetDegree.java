package frc.robot.commands.Shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterSetDegree extends PIDCommand {

  public ShooterSetDegree(ShooterSubsystem m_shooter, DoubleSupplier angle) {
    super(

        new PIDController(0.04,
        //new PIDController(0.025,

            0.03,
            Constants.values.shooter.PidShooterAngleKD),
        () -> m_shooter.getMappedOutput(),
        () -> angle.getAsDouble(),
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
