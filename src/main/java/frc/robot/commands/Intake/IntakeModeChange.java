// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.JoystickSubsystem;

public class IntakeModeChange extends Command {

int mode;
JoystickSubsystem m_joystick;

  public IntakeModeChange(JoystickSubsystem m_joystick,int mode) {
    this.m_joystick = m_joystick; 
    this.mode = mode;
  }
  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_joystick.intakemodchange(mode);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
