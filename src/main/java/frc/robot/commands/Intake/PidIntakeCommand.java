package frc.robot.commands.Intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.Constants.controller;
import frc.robot.subsystems.IntakeSubsystem;

public class PidIntakeCommand extends PIDCommand {
  double tolerance = 5;

  public PidIntakeCommand(IntakeSubsystem m_intake, double position) {
    super(

        new PIDController(0.05,
            Constants.values.intake.PidIntakeKI,
            Constants.values.intake.PidIntakeKD),
        () -> m_intake.getRawEncoderOutput(),
        () -> position,

        output -> {
          if (position > -m_intake.getRawEncoderOutput()) {
            m_intake.NewIntakeMotorOutput(output * 0.8);
          }

        else if (position < -m_intake.getRawEncoderOutput()) {
            m_intake.NewIntakeMotorOutput(-output * 0.8);
          }

        });
    addRequirements(m_intake);
    getController().setTolerance(Constants.values.intake.PidIntakeTolerance);
  }

  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
