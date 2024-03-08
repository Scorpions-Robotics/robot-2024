// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.common;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Intake.IntakeModeChange;
import frc.robot.commands.Intake.PidIntakeCommand;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.JoystickSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class dalhacan extends SequentialCommandGroup {
  /** Creates a new dalhacan. */
  public dalhacan(IntakeSubsystem m_intake, ShooterSubsystem m_shooter,JoystickSubsystem m_joystick) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new IntakeModeChange(m_joystick, 1)
        .andThen(new InstantCommand(() -> m_shooter.ShootertoFeederPos()))
            //.alongWith(new IntakeModeChange(m_joystick, 1))

    );

  }
}
