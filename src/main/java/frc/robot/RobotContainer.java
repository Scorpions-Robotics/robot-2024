package frc.robot;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.proto.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Intake.PidIntakeCommand;
import frc.robot.commands.Shooter.PidSetRpm;
import frc.robot.commands.Shooter.ShooterSetDegree;
import frc.robot.commands.Swerve.SwerveJoystickCmd;
import frc.robot.commands.common.FeedingPosition;
import frc.robot.commands.common.IntakeInputPosition;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;    

public class RobotContainer {
        private final IntakeSubsystem m_intake = new IntakeSubsystem(); 
        private final ShooterSubsystem m_shooter = new ShooterSubsystem(); 
   private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final XboxController driverJoytick = new XboxController(1);  
    private final FeederSubsystem m_feeder = new FeederSubsystem();

  public RobotContainer() {

    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> -driverJoytick.getRawAxis(OIConstants.kDriverYAxis),
                () -> driverJoytick.getRawAxis(OIConstants.kDriverXAxis),
                () -> driverJoytick.getRawAxis(OIConstants.kDriverRotAxis),
                () -> !driverJoytick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));

            configureBindings();
    
  }




  private void configureBindings() {
    //new JoystickButton(driverJoytick, 2).whileTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
    
    //new JoystickButton(driverJoytick, 3).whileTrue(new PidIntakeCommand(m_intake, 20));
    //new JoystickButton(driverJoytick, 3).whileFalse(new InstantCommand(() -> m_intake.StopAngleMotor()));
    //new JoystickButton(driverJoytick, 4).whileTrue(new InstantCommand(()-> m_intake.reset()));
    //new JoystickButton(driverJoytick, 5).whileTrue(new InstantCommand(()-> m_shooter.AngleEncoderReset()));
    //new JoystickButton(driverJoytick, 6).whileTrue(new InstantCommand(()-> m_feeder.backward()));
  // new JoystickButton(driverJoytick, 3).whileTrue(new InstantCommand(()->m_shooter.ShooterThrowMotorOutput(0.7)));

   //new JoystickButton(driverJoytick, 3).whileFalse(new InstantCommand(()->m_shooter.ShooterThrowAllMotorStop()));
    new JoystickButton(driverJoytick, 1).whileTrue(new IntakeInputPosition(m_intake));
    new JoystickButton(driverJoytick, 3).whileTrue(new ShooterSetDegree(m_shooter, 50.0));
    new JoystickButton(driverJoytick, 4).whileTrue(new FeedingPosition(m_shooter, m_intake));

    new JoystickButton(driverJoytick, 1).whileFalse(new InstantCommand(()->m_intake.StopAngleMotor()));
    new JoystickButton(driverJoytick, 2).whileTrue(new InstantCommand(()->m_intake.reset()));
    new JoystickButton(driverJoytick, 2).whileTrue(new InstantCommand(()->m_shooter.AngleEncoderReset()));

   // new JoystickButton(driverJoytick, 1).whileTrue(new InstantCommand(()->m_shooter.AngleEncoderReset()));
    //new JoystickButton(driverJoytick, 2).whileTrue(new IntakeInputPosition(m_intake));
    //new JoystickButton(driverJoytick, 3).whileTrue(new FeedingPosition(m_shooter,m_intake));
  //  new JoystickButton(driverJoytick, 4).whileTrue(new RunCommand(()-> m_feeder.forward()));
   // new JoystickButton(driverJoytick, 5).whileTrue(new RunCommand(()-> m_feeder.backward()));
   // new JoystickButton(driverJoytick, 4).whileFalse(new RunCommand(()-> m_feeder.stop()));
    new JoystickButton(driverJoytick, 5).whileTrue(new InstantCommand(()-> m_intake.runpickupmotor(0.5)));
    new JoystickButton(driverJoytick, 5).whileFalse(new InstantCommand(()-> m_intake.runpickupmotor(0.0)));

    new JoystickButton(driverJoytick, 6).whileTrue(new InstantCommand(()-> m_feeder.backward()));
    new JoystickButton(driverJoytick, 6).whileFalse(new InstantCommand(()->  m_feeder.stop()));

  }



  public Command getAutonomousCommand() {
/* // 1. Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(DriveConstants.kDriveKinematics);

        // 2. Generate trajectory
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(
                        new Translation2d(1, 0),
                        new Translation2d(1, -1)),
                new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
                trajectoryConfig);

        // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // 4. Construct command to follow trajectory
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
                swerveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                swerveSubsystem::setModuleStates,
                swerveSubsystem);

        // 5. Add some init and wrap-up, and return everything
        return new SequentialCommandGroup(
                new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
                swerveControllerCommand,
                new InstantCommand(() -> swerveSubsystem.stopModules())); */
                return null;
    }  }
