// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class otoFeederRun extends Command {
  FeederSubsystem m_feeder;
  boolean dursunmu;
  IntakeSubsystem m_intake;


  public otoFeederRun(FeederSubsystem m_feeder, boolean dursunmu, IntakeSubsystem m_intake) {
    this.m_feeder = m_feeder;
    this.m_intake = m_intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


if(m_feeder.varmi){
if(m_feeder.detector()){
m_feeder.otobackward2();
}
else{
  m_feeder.stop();
  m_intake.StopNoteMotor();
  m_feeder.varmifalse();

}}






}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {} 

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return dursunmu;
  }

}
