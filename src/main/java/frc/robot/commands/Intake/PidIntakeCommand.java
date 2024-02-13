package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.IntakeSubsystem;


public class PidIntakeCommand extends PIDCommand {
  
  public PidIntakeCommand(IntakeSubsystem m_intake, double position) {
    super(
       
        new PIDController(1, 0, 0),
        
        () -> m_intake.getEncoderOutput(),
        
        () -> position,
        
        output -> {
          if (position > m_intake.getEncoderOutput()) {
            m_intake.intakeMotorOutput(-output);;
          } else if (position < m_intake.getEncoderOutput()) {
            m_intake.intakeMotorOutput(output);;
          }
          
        });
    
  }

  
  @Override
  public boolean isFinished() {
    return false;
  }
}
