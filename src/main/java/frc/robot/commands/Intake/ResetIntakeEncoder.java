package frc.robot.commands.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class ResetIntakeEncoder extends Command {
  
    IntakeSubsystem m_intake;

    public ResetIntakeEncoder(IntakeSubsystem m_intake){
      this.m_intake = m_intake;
    }

  @Override
  public void initialize() {}
 
  @Override
  public void execute() {
    m_intake.reset();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
