// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;

public class FeederRunTillSwitch extends Command {
  FeederSubsystem m_feeder;
  boolean dursunmu;


  public FeederRunTillSwitch(FeederSubsystem m_feeder, boolean dursunmu) {

    this.m_feeder = m_feeder;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


    boolean ilkswitch;

    ilkswitch = m_feeder.detector();

if(m_feeder.varmi){


if(!dursunmu){

  if(ilkswitch){
    
    m_feeder.backward2();
    
        }else{
          m_feeder.varmifalse();
          m_feeder.stop();
        }
      }
}


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
