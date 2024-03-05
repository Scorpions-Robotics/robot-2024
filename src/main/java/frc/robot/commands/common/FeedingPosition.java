package frc.robot.commands.common;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Intake.PidIntakeCommand;
import frc.robot.commands.Shooter.ShooterSetDegree;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class FeedingPosition extends SequentialCommandGroup {
  public FeedingPosition(ShooterSubsystem m_shooter, IntakeSubsystem m_intake) {

    addCommands(
    new PidIntakeCommand(m_intake,1.2)
    .alongWith(new ShooterSetDegree(m_shooter, -7.6))
    );
  }
}
