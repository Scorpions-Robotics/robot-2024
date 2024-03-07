
package frc.robot.commands.common;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Intake.PidIntakeCommand;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class cinarcan extends SequentialCommandGroup {
  
  public cinarcan(IntakeSubsystem m_intake, FeederSubsystem m_feeder) {
   
    addCommands(
    new PidIntakeCommand(m_intake, 1.5)
    .andThen(new InstantCommand(()-> m_feeder.runtillswitch()).alongWith(new InstantCommand(()-> m_intake.NewIntakeMotorOutput(0.7))))

    );
  }
}
