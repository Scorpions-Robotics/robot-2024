// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Shooter.ShooterSetDegree;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoNoteShoot extends SequentialCommandGroup {
  /** Creates a new AutoNoteShoot. */
  public AutoNoteShoot(ShooterSubsystem m_shooter, FeederSubsystem m_feeder, DoubleSupplier degree) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands( 
      new ShooterSetDegree(m_shooter, degree).
      withTimeout(1).
      alongWith(new InstantCommand(() -> m_shooter.ShooterThrowMotorOutput(-0.6))
      .andThen(new WaitCommand(0.6))
      .andThen(new InstantCommand(() -> m_feeder.backward()).withTimeout(0.5)))
      .andThen(new InstantCommand(() -> m_shooter.ShooterThrowMotorOutput(0))
      .andThen(new InstantCommand(() -> m_feeder.stop())))
    );
  }
}
