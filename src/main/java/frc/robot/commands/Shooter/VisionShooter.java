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
            //0.01,
            0.003,
            Constants.values.shooter.PidShooterAngleKD),
        () -> m_network.getY(),
        () -> m_network.getTarget(),
        output -> {
         /*  try {
            m_shooter.ShooterAngleMotorOutput(output * .13);
            Thread.sleep(150); // 2 saniye durakla
            m_shooter.ShooterAn
            gleMotorStop();
          } catch (InterruptedException e) {
            e.printStackTrace();
        }*/

        m_shooter.ShooterAngleMotorOutput(output * .13);

 // m_shooter.ShooterAngleMotorOutput(output * .09);
});

    addRequirements(m_shooter);
    getController().setTolerance(5);

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
