// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.common;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Intake.PidIntakeCommand;
import frc.robot.commands.feeder.FeederRunTillSwitch;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class fedleme extends SequentialCommandGroup {
  /** Creates a new fedleme. */
  public fedleme(IntakeSubsystem m_intake, FeederSubsystem m_feeder) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
new PidIntakeCommand(m_intake, 1.8)

.andThen(new RunCommand(()-> m_intake.getNote())
.andThen(new FeederRunTillSwitch(m_feeder, false)
))
.withTimeout(2)




    );
  }
}
