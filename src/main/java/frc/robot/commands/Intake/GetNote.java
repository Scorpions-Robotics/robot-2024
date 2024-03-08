// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class GetNote extends Command {
  IntakeSubsystem m_intake;
  public GetNote(IntakeSubsystem m_intake) {
  this.m_intake = m_intake;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_intake.getNote();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
