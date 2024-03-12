// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.common;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Intake.IntakeModeChange;
import frc.robot.commands.Intake.RunTillSwitch;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.JoystickSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class pickup extends SequentialCommandGroup {
  /** Creates a new pickup. */
  public pickup(IntakeSubsystem m_intake, JoystickSubsystem m_joystick) {
    addCommands(new RunTillSwitch(m_intake,false).alongWith(new IntakeModeChange(m_joystick, 1)));
  }
}
