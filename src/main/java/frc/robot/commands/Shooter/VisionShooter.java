package frc.robot.commands.Shooter;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.NetworkSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class VisionShooter extends PIDCommand {

  public VisionShooter(ShooterSubsystem m_shooter, NetworkSubsystem m_network) {
    super(

        new PIDController(0.004,
            0,
            Constants.values.shooter.PidShooterAngleKD),
        () -> m_network.getY(),
        () -> -165,
        output -> {
         /*  try {
            m_shooter.ShooterAngleMotorOutput(output * .13);
            Thread.sleep(150); // 2 saniye durakla
            m_shooter.ShooterAngleMotorStop();
          } catch (InterruptedException e) {
            e.printStackTrace();
        }*/






  m_shooter.ShooterAngleMotorOutput(output * .13);
});

    addRequirements(m_shooter);
    getController().setTolerance(5);

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
