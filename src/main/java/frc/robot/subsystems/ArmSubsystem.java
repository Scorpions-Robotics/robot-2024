// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
CANSparkMax armleft = new CANSparkMax(16, MotorType.kBrushless);
CANSparkMax armright = new CANSparkMax(13, MotorType.kBrushless);

  public ArmSubsystem() {
    armleft.setIdleMode(IdleMode.kBrake);
    armright.setIdleMode(IdleMode.kBrake);

  }


  public void rightarmup(){
    armright.set(0.6);
  }

  public void rightarmdown(){
    armright.set(-0.6);

  }
  
  public void leftarmup(){
    armleft.set(-0.6);

  }

  public void leftarmdown(){
    armleft.set(0.6);

  }
  public void leftarmstop(){
    armleft.set(0);

  }
  public void rightarmstop(){
    armright.set(0);

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
