// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.common;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Intake.GetNote;
import frc.robot.commands.Intake.IntakeModeChange;
import frc.robot.commands.Intake.PidIntakeCommand;
import frc.robot.commands.Intake.RunTillSwitch;
import frc.robot.commands.Shooter.ShooterModeChange;
import frc.robot.commands.feeder.FeederRunTillSwitch;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.JoystickSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class fedleme extends SequentialCommandGroup {
  /** Creates a new fedleme. */
  public fedleme(IntakeSubsystem m_intake, FeederSubsystem m_feeder, JoystickSubsystem m_joystick, ShooterSubsystem m_shooter) {

    addCommands(
      new InstantCommand(()-> m_shooter.ShootertoFeederPos())
      .andThen(new IntakeModeChange(m_joystick, 0))
      .alongWith(new WaitCommand(1).andThen(new GetNote(m_intake)))
      .alongWith(new FeederRunTillSwitch(m_feeder, false, m_intake))



    );
  }
}
