package frc.robot.commands.Intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class PidIntakeCommand extends PIDCommand {
  double tolerance = 5;
  
  public PidIntakeCommand(IntakeSubsystem m_intake, double position) {
    super(
       
        new PIDController(Constants.values.intake.PidIntakeKP,
         Constants.values.intake.PidIntakeKI,
          Constants.values.intake.PidIntakeKD),
        () -> m_intake.getMappedOutput(),
        () -> position,
        
        output -> {
          if (position > m_intake.getMappedOutput()) {
            m_intake.NewIntakeMotorOutput(-output);
          } 
          
          else if (position < m_intake.getMappedOutput()) {
            m_intake.NewIntakeMotorOutput(output);
          }
          
        });
        addRequirements(m_intake);
        getController().setTolerance(Constants.values.intake.PidIntakeTolerance);
  }

  
  @Override
  public boolean isFinished() {
    return false;
  }
}
