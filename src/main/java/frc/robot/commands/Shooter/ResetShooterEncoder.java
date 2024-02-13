package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ResetShooterEncoder extends Command {

  ShooterSubsystem m_shooter;

  public ResetShooterEncoder(ShooterSubsystem m_shooter) {
    this.m_shooter = m_shooter;
  }

  @Override
  public void initialize() {}


  @Override
  public void execute() {
  m_shooter.AngleEncoderReset();
  }


  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
