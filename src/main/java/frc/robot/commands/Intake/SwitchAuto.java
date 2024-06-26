// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.common.fedleme;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.JoystickSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
 
public class SwitchAuto extends Command {
  IntakeSubsystem m_intake;
  boolean dursunmu;
  FeederSubsystem m_feeder;
  JoystickSubsystem m_joystick;
  ShooterSubsystem m_shooter;
boolean varmi = false;


  public SwitchAuto(IntakeSubsystem m_intake, boolean dursunmu) {

    this.m_intake = m_intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


    boolean ilkswitch;
    boolean ikinciswitch;

    ilkswitch = m_intake.intakedetector1.get();
    ikinciswitch = m_intake.intakedetector2.get();






if(!dursunmu){

  if(ikinciswitch && ilkswitch){

    m_intake.getNote();
    
        }else{
    
          m_intake.StopNoteMotor();
          varmi = true;
        }
      }



    //  if(varmi){
    //    m_intake.getNote();
    //  }



    SmartDashboard.putBoolean("asdadssdaasd", ilkswitch);
    SmartDashboard.putBoolean("asdadssdaasd2222", ikinciswitch);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_feeder.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return varmi;
  }

}
