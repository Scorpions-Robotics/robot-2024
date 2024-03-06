package frc.robot.commands.common;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Intake.PidIntakeCommand;
import frc.robot.commands.Shooter.ShooterSetDegree;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class FeedingPosition extends SequentialCommandGroup {
  public FeedingPosition(ShooterSubsystem m_shooter,FeederSubsystem m_feeder , IntakeSubsystem m_intake) {

    addCommands(
      //1.8
     ( new PidIntakeCommand(m_intake, 1.2)
      
    //-3.8
    .andThen(
      new InstantCommand(()->m_feeder.runtillswitch()).alongWith(new InstantCommand(() -> m_intake.NewIntakeMotorOutput(0.4)))
      //ShooterSetDegree(m_shooter, -1.15)
      
    )
      
      
      )
    
    
    );
  }
}
