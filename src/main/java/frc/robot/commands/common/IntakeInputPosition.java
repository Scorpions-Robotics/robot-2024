
package frc.robot.commands.common;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Intake.PidIntakeCommand;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeInputPosition extends SequentialCommandGroup {
  public IntakeInputPosition(IntakeSubsystem m_intake) {
    addCommands(new PidIntakeCommand(m_intake, ()->12.0));
  }
}
