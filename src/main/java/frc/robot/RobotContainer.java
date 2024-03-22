package frc.robot;

import java.util.List;

import javax.management.RuntimeErrorException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
//import edu.wpi.first.math.proto.Trajectory;
//import edu.wpi.first.math.proto.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
//import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Intake.GetNote;
import frc.robot.commands.Intake.IntakeModeChange;
import frc.robot.commands.Intake.PidIntakeCommand;
import frc.robot.commands.Intake.RunTillSwitch;
import frc.robot.commands.Shooter.PidSetRpm;
import frc.robot.commands.Shooter.ShooterModeChange;
import frc.robot.commands.Shooter.ShooterSet180;
import frc.robot.commands.Shooter.ShooterSetDegree;
import frc.robot.commands.Shooter.TurnForShooter;
import frc.robot.commands.Shooter.VisionShooter;
import frc.robot.commands.Swerve.SwerveJoystickCmd;
import frc.robot.commands.autonomous.AutoNoteShoot;
import frc.robot.commands.common.FeedingPosition;
import frc.robot.commands.common.IntakeInputPosition;
import frc.robot.commands.common.dalhacan;
import frc.robot.commands.common.fedleme;
import frc.robot.commands.common.otofeedleme;
import frc.robot.commands.common.pickup;
import frc.robot.commands.common.pickupforAuto;
import frc.robot.commands.feeder.FeederRunTillSwitch;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.JoystickSubsystem;
import frc.robot.subsystems.NetworkSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem; 
//import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.Trajectory;


public class RobotContainer {
        public final IntakeSubsystem m_intake = new IntakeSubsystem();
        public final ShooterSubsystem m_shooter = new ShooterSubsystem();
        public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
        public final XboxController driverJoytick = new XboxController(1);
        public final XboxController subJoytick = new XboxController(3);
        public final FeederSubsystem m_feeder = new FeederSubsystem();
        public final ArmSubsystem m_arm = new ArmSubsystem();
        public final JoystickSubsystem m_joystick = new JoystickSubsystem();
        NetworkSubsystem m_network = new NetworkSubsystem();
        
        public int autonomous_case;

        public RobotContainer() {
                swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                                swerveSubsystem,
                                () -> driverJoytick.getRawAxis(OIConstants.kDriverYAxis),
                                () -> -driverJoytick.getRawAxis(OIConstants.kDriverXAxis),
                                () -> driverJoytick.getRawAxis(OIConstants.kDriverRotAxis),
                                () -> !driverJoytick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));

        m_intake.setDefaultCommand(new PidIntakeCommand(m_intake,() -> m_joystick.getintakevalue()));
        //m_shooter.setDefaultCommand(new ShooterSetDegree(m_shooter, () -> m_joystick.getshootervalue()));



                configureBindings();

        }

        private void configureBindings() {
/*  
 ____  ____  __  _  _  ____  ____ 
(    \(  _ \(  )/ )( \(  __)(  _ \
 ) D ( )   / )( \ \/ / ) _)  )   /
(____/(__\_)(__) \__/ (____)(__\_) */




//buton 1
new JoystickButton(driverJoytick, 1).whileTrue(new InstantCommand(()-> m_feeder.forward()));
new JoystickButton(driverJoytick, 1).whileFalse(new InstantCommand(()-> m_feeder.stop()));
new JoystickButton(driverJoytick, 1).whileTrue(new InstantCommand(()-> m_shooter.ShooterThrowMotorOutput(0.5)));
new JoystickButton(driverJoytick, 1).whileFalse(new InstantCommand(()-> m_shooter.ShooterThrow1MotorStop()));
new JoystickButton(driverJoytick, 1).whileFalse(new InstantCommand(()-> m_shooter.ShooterThrow2MotorStop()));


//buton 2
new JoystickButton(driverJoytick, 2).whileTrue(new InstantCommand(()-> m_feeder.backward()));

new JoystickButton(driverJoytick, 2).whileFalse(new InstantCommand(()-> m_feeder.stop()));


//button 3
// TIRMANMA new JoystickButton(driverJoytick, 1).whileTrue(new InstantCommand(()-> m_feeder.forward()));

new JoystickButton(driverJoytick, 5).whileTrue(new InstantCommand(()-> m_arm.leftarmup()));
new JoystickButton(driverJoytick, 5).whileFalse(new InstantCommand(()-> m_arm.leftarmstop()));

new JoystickButton(driverJoytick, 6).whileTrue(new InstantCommand(()-> m_arm.rightarmup()));
new JoystickButton(driverJoytick, 6).whileFalse(new InstantCommand(()-> m_arm.rightarmstop()));


new JoystickButton(driverJoytick, 7).whileTrue(new InstantCommand(()-> m_arm.leftarmdown()));
new JoystickButton(driverJoytick, 7).whileFalse(new InstantCommand(()-> m_arm.leftarmstop()));


new JoystickButton(driverJoytick, 8).whileTrue(new InstantCommand(()-> m_arm.rightarmdown()));
new JoystickButton(driverJoytick, 8).whileFalse(new InstantCommand(()-> m_arm.rightarmstop()));
new JoystickButton(driverJoytick, 10).whileTrue(new InstantCommand(()-> swerveSubsystem.resetGyro()));
new JoystickButton(driverJoytick, 10).whileTrue(new InstantCommand(()-> swerveSubsystem.resetOdometry()));


/* 
 ____  _  _  ____  ____  ____  __  _  _  ____  ____
/ ___)/ )( \(  _ \(    \(  _ \(  )/ )( \(  __)(  _ \
\___ \) \/ ( ) _ ( ) D ) )   / )( \ \/ / ) _)  )   /
(____/\____/(____/(____/(__\_)(__) \__/ (____)(__\_)
*/
//buton 1
new JoystickButton(subJoytick, 1).whileTrue(new pickup(m_intake, m_joystick));
new JoystickButton(subJoytick, 1).whileFalse(new InstantCommand(()-> m_intake.StopNoteMotor()));


//buton 2
new JoystickButton(subJoytick, 2).whileTrue(new fedleme(m_intake, m_feeder, m_joystick,m_shooter));
new JoystickButton(subJoytick, 2).whileTrue(new InstantCommand(()->m_feeder.varmitrue()));
new JoystickButton(subJoytick, 2).whileFalse(new InstantCommand(()->m_feeder.stop()));
new JoystickButton(subJoytick, 2).whileFalse(new InstantCommand(()->m_intake.StopNoteMotor()));

//buton 3 Shooter Manuel subbuffer
new JoystickButton(subJoytick, 3).whileTrue(new VisionShooter(m_shooter, m_network));
new JoystickButton(subJoytick, 3).whileTrue(new TurnForShooter(m_network, swerveSubsystem));
//new JoystickButton(subJoytick, 3).whileTrue(new InstantCommand(() -> m_joystick.shooterdegistiraktif()));
//new JoystickButton(subJoytick, 3).whileTrue(new InstantCommand(() -> m_joystick.shooterkayma(m_shooter.getMappedOutput())));
//new JoystickButton(subJoytick, 7).whileTrue(new ShooterSetDegree(m_shooter, () -> m_joystick.returnshooterdegeri()));
new JoystickButton(subJoytick, 7).whileTrue(new InstantCommand(()-> m_shooter.ShooterThrowMotorOutput(-1)));
new JoystickButton(subJoytick, 7).whileFalse(new InstantCommand(()-> m_shooter.ShooterThrow1MotorStop()));
new JoystickButton(subJoytick, 7).whileFalse(new InstantCommand(()-> m_shooter.ShooterThrow2MotorStop()));

//button 4 intake push
new JoystickButton(subJoytick, 4).whileTrue(new ShooterSetDegree(m_shooter, ()-> 82.0));
new JoystickButton(subJoytick, 4).whileTrue(new InstantCommand(()-> m_shooter.ShooterThrowMotorOutput(0.5)));
new JoystickButton(subJoytick, 4).whileFalse(new InstantCommand(()-> m_shooter.ShooterThrow1MotorStop()));
 new JoystickButton(subJoytick, 4).whileFalse(new InstantCommand(()-> m_shooter.ShooterThrow2MotorStop()));
//button 5 Amfi 
 new JoystickButton(subJoytick, 5).whileTrue(new ShooterSet180(m_shooter));
 new JoystickButton(subJoytick, 6).whileTrue(new ShooterSetDegree(m_shooter, ()->73.0));
 new JoystickButton(subJoytick, 6).whileTrue(new InstantCommand(()-> m_shooter.ShooterThrowMotorOutput(-1)));
 new JoystickButton(subJoytick, 6).whileFalse(new InstantCommand(()-> m_shooter.ShooterThrow1MotorStop()));
 new JoystickButton(subJoytick, 6).whileFalse(new InstantCommand(()-> m_shooter.ShooterThrow2MotorStop()));

 new JoystickButton(subJoytick, 8).whileTrue(new ShooterSetDegree(m_shooter, ()->160.0));
 new JoystickButton(subJoytick, 8).whileTrue(new InstantCommand(()-> m_shooter.ShooterThrowMotorOutput(-1)));
 new JoystickButton(subJoytick, 8).whileFalse(new InstantCommand(()-> m_shooter.ShooterThrow1MotorStop()));
 new JoystickButton(subJoytick, 8).whileFalse(new InstantCommand(()-> m_shooter.ShooterThrow2MotorStop()));

new JoystickButton(subJoytick, 9).whileTrue(new InstantCommand(()-> m_intake.pushNote()));
new JoystickButton(subJoytick, 9).whileFalse(new InstantCommand(()-> m_intake.StopNoteMotor()));








//SUBDRIVER









                //Driver
               // new JoystickButton(driverJoytick, 8).whileTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
                
                // new JoystickButton(driverJoytick, 6).whileTrue(new InstantCommand(()-> m_intake.pushNote()));
                // new JoystickButton(driverJoytick, 6).whileFalse(new InstantCommand(()->m_intake.StopNoteMotor()));


              //  new JoystickButton(driverJoytick, 7).whileTrue(new InstantCommand(()-> m_joystick.shootermodchange(0)));
                //new JoystickButton(driverJoytick, 8).whileTrue(new InstantCommand(()-> m_joystick.shootermodchange(1)));

        //      new JoystickButton(subJoytick, 4).whileTrue(new fedleme(m_intake, m_feeder, m_joystick,m_shooter));
        //      new JoystickButton(driverJoytick, 4).whileTrue(new InstantCommand(()->m_feeder.varmitrue()));
        //      new JoystickButton(driverJoytick, 4).whileFalse(new InstantCommand(()->m_feeder.stop()));
        //      new JoystickButton(driverJoytick, 4).whileFalse(new InstantCommand(()->m_intake.StopNoteMotor()));



             //new JoystickButton(driverJoytick, 1).whileTrue(new InstantCommand(()-> m_joystick.intakemodchange(1)));
              // new JoystickButton(driverJoytick, 5).whileTrue(new RunTillSwitch(m_intake,false,m_feeder,m_joystick,m_shooter));
                //new JoystickButton(driverJoytick, 5).whileFalse(new InstantCommand(()->m_intake.StopNoteMotor()));
                // new JoystickButton(subJoytick, 3).whileTrue(new ShooterSetDegree(m_shooter, ()->70.0));

            //  new JoystickButton(driverJoytick, 6).whileTrue(new FedX(m_intake, m_feeder, m_joystick,m_shooter));

               
            //new JoystickButton(driverJoytick, 6).whileFalse(new InstantCommand(()->m_intake.StopNoteMotor()));

            //new JoystickButton(driverJoytick, 6).whileTrue(new InstantCommand(()->m_intake.pushNote()));
           // new JoystickButton(driverJoytick, 6).whileFalse(new InstantCommand(()->m_intake.StopNoteMotor()));

                //new JoystickButton(driverJoytick, 5).whileTrue(new InstantCommand(() -> m_feeder.backward()));
                //new JoystickButton(driverJoytick, 5).whileTrue(new InstantCommand(() -> m_feeder.varmitrue()));

               // new JoystickButton(driverJoytick, 5).whileFalse(new InstantCommand(() -> m_feeder.stop()));
               
               
               
               
               //sub
               // new JoystickButton(subJoytick, 1).whileTrue(new InstantCommand(()->m_joystick.intakemodchange(1)));

             //   new JoystickButton(subJoytick, 6).whileTrue(new fedleme(m_intake, m_feeder, m_joystick,m_shooter));

             //new JoystickButton(subJoytick, 6).whileFalse(new InstantCommand(()->m_intake.StopNoteMotor()));
                //new JoystickButton(subJoytick, 6).whileFalse(new InstantCommand(()->m_feeder.stop()));

                //new JoystickButton(subJoytick, 6).whileTrue(new FeederRunTillSwitch(m_feeder, false));
                //new JoystickButton(subJoytick, 6).whileFalse(new InstantCommand(() -> m_feeder.stop()));

                //new JoystickButton(subJoytick, 2).whileTrue(new IntakeModeChange(m_joystick, 0));
                //new JoystickButton(subJoytick, 2).whileTrue(new ShooterSetDegree(m_shooter, 30));
                
//once                 new JoystickButton(subJoytick, 2).whileTrue(new PidIntakeCommand(m_intake,1.3));

//sonra                 

                //new JoystickButton(subJoytick, 5).whileTrue(new GetNote(m_intake));
              //  new JoystickButton(subJoytick, 5).whileFalse(new InstantCommand(()->m_intake.StopNoteMotor()));

               // new JoystickButton(subJoytick, 3).whileTrue(new ShooterSetDegree(m_shooter, ()->70.0));
                
             //   new JoystickButton(subJoytick, 4)
               //           .whileTrue(new InstantCommand(() -> m_shooter.ShooterThrowMotorOutput(0.8)));

             //   new JoystickButton(subJoytick, 4)
               //                .whileFalse(new InstantCommand(() -> m_shooter.ShooterThrowAllMotorStop()));
                                
                //new JoystickButton(subJoytick, 7).whileTrue(new InstantCommand(() -> m_intake.reset()));
               // new JoystickButton(driverJoytick, 4).whileTrue(new InstantCommand(()->m_shooter.ShootertoFeederPos()));

                // new JoystickButton(driverJoytick, 3).whileTrue(new PidIntakeCommand(m_intake,
                // 20));
                // new JoystickButton(driverJoytick, 3).whileFalse(new InstantCommand(() ->
                // m_intake.StopAngleMotor()));
                // new JoystickButton(driverJoytick, 4).whileTrue(new InstantCommand(()->
                // m_intake.reset()));
                // new JoystickButton(driverJoytick, 5).whileTrue(new InstantCommand(()->
                // m_shooter.AngleEncoderReset()));
                // new JoystickButton(driverJoytick, 6).whileTrue(new InstantCommand(()->
                // m_feeder.backward()));

        //         new JoystickButton(driverJoytick, 10)
        //                         .whileTrue(new InstantCommand(() -> m_shooter.ShooterThrowMotorOutput(0.8)));

        //         new JoystickButton(driverJoytick, 10)
        //                         .whileFalse(new InstantCommand(() -> m_shooter.ShooterThrowAllMotorStop()));

        //         new JoystickButton(driverJoytick, 1).whileTrue(new InstantCommand(() -> m_intake.degistirmee()));
        //         new JoystickButton(driverJoytick, 3).whileTrue(new ShooterSetDegree(m_shooter, 160.0));
        //         // new JoystickButton(driverJoytick, 4).whileTrue(new cinarcan(m_intake,
        //         // m_feeder));
            //  new JoystickButton(driverJoytick, 4).whileTrue(new RunTillSwitch(m_intake));
        //        new JoystickButton(driverJoytick, 4).whileFalse(new InstantCommand(() -> m_intake.StopNoteMotor()));

        //         new JoystickButton(driverJoytick, 1).whileFalse(new InstantCommand(() -> m_intake.StopAngleMotor()));
        //         new JoystickButton(driverJoytick, 2).whileTrue(new InstantCommand(() -> m_intake.reset()));
        //         new JoystickButton(driverJoytick, 2).whileTrue(new InstantCommand(() -> m_shooter.AngleEncoderReset()));

                // new JoystickButton(driverJoytick, 1).whileTrue(new
                // InstantCommand(()->m_shooter.AngleEncoderReset()));
                // new JoystickButton(driverJoytick, 2).whileTrue(new
                // IntakeInputPosition(m_intake));
                // new JoystickButton(driverJoytick, 3).whileTrue(new
                // FeedingPosition(m_shooter,m_intake));
                // new JoystickButton(driverJoytick, 4).whileTrue(new RunCommand(()->
                // m_feeder.forward()));
                // new JoystickButton(driverJoytick, 5).whileTrue(new RunCommand(()->
                // m_feeder.backward()));
                // new JoystickButton(driverJoytick, 4).whileFalse(new RunCommand(()->
                // m_feeder.stop()));
                //new JoystickButton(driverJoytick, 9).whileTrue(new cinarcan(m_intake, m_feeder));
                //ew JoystickButton(driverJoytick, 9).whileFalse(new cinarcan(m_intake, m_feeder));

   // new JoystickButton(driverJoytick, 1).whileTrue(new InstantCommand(()->m_shooter.AngleEncoderReset()));
    //new JoystickButton(driverJoytick, 2).whileTrue(new IntakeInputPosition(m_intake));
    //new JoystickButton(driverJoytick, 3).whileTrue(new FeedingPosition(m_shooter,m_intake));
  //  new JoystickButton(driverJoytick, 4).whileTrue(new RunCommand(()-> m_feeder.forward()));
   // new JoystickButton(driverJoytick, 5).whileTrue(new RunCommand(()-> m_feeder.backward()));
   // new JoystickButton(driverJoytick, 4).whileFalse(new RunCommand(()-> m_feeder.stop()));
   // new JoystickButton(driverJoytick, 9).whileTrue(new cinarcan(m_intake, m_feeder));
    //new JoystickButton(driverJoytick, 9).whileFalse(new cinarcan(m_intake, m_feeder));
//     new JoystickButton(driverJoytick, 5).whileTrue(new RunTillSwitch(m_intake,false));
//     new JoystickButton(driverJoytick, 5).whileFalse(new InstantCommand(()->m_intake.StopNoteMotor()));

    
   // new JoystickButton(driverJoytick, 5).onTrue(new InstantCommand(()-> m_intake.degistir()));

                // new JoystickButton(driverJoytick, 6).whileTrue(new FeederRunTillSwitch(m_feeder, false));
                // new JoystickButton(driverJoytick, 6).whileFalse(new InstantCommand(() -> m_feeder.stop()));

                // new JoystickButton(driverJoytick, 8).whileTrue(new InstantCommand(() -> m_intake.runpickupmotor(0.8)));
                // new JoystickButton(driverJoytick, 8).whileFalse(new InstantCommand(() -> m_intake.runpickupmotor(0)));
               
                // new JoystickButton(driverJoytick, 9).whileFalse(new InstantCommand(() -> m_intake.getNote()));

        
        }

        public Command getAutonomousCommand(int mode) {

                // TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                //         AutoConstants.kMaxSpeedMetersPerSecond,
                //         AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                //                 .setKinematics(DriveConstants.kDriveKinematics);
        
                PIDController xController = new PIDController(0.08, 0, 0);
                PIDController yController = new PIDController(0.08, 0, 0);
                
                ProfiledPIDController thetaController = new ProfiledPIDController(
                        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
                thetaController.enableContinuousInput(-Math.PI, Math.PI);
        
                TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                        4,
                        3)
                                .setKinematics(DriveConstants.kDriveKinematics);
        
                // Pathler

                var trajectoryOne =
                        TrajectoryGenerator.generateTrajectory(
                          List.of(new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
                          new Pose2d(-1.28, -0.02, new Rotation2d(0.0)),
                          new Pose2d(0.0, 0.0, new Rotation2d(0.0))),
                        trajectoryConfig);

                        var trajectory5 =
                        TrajectoryGenerator.generateTrajectory(
                          List.of(new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
                          new Pose2d(-2.28, -0.02, new Rotation2d(0.0))),
                        trajectoryConfig);
                
                // var trajectoryOne =
                //         TrajectoryGenerator.generateTrajectory(
                //           new Pose2d(0, 0, new Rotation2d(0)),
                //           List.of(new Translation2d(-1.28, 0),
                //           new Translation2d(-1.28, 0),
                //           new Translation2d(-1.28, 0)),
                //           new Pose2d(0, 0, new Rotation2d(0)),
                //         trajectoryConfig);
                                
                var trajectoryTwo =
                        TrajectoryGenerator.generateTrajectory(
                          new Pose2d(0, 0, new Rotation2d(0)),
                          List.of(new Translation2d(-0.4, -0.3),new Translation2d(-2, 0.0),
                          new Translation2d(0.0, 0.0)),
                          new Pose2d(-0.1,0, new Rotation2d(0)),
                        trajectoryConfig);
                        

                        var note3pathforblue =
                        TrajectoryGenerator.generateTrajectory(
                          new Pose2d(0, 0, new Rotation2d(0)),
                          List.of(new Translation2d(-0.5, 1),new Translation2d(-1.0, 1.6),
                          new Translation2d(-1.58, 1.6)
                          ),
                          new Pose2d(-1.28,0,new Rotation2d(0)),
                        trajectoryConfig);

                var note3path =
                        TrajectoryGenerator.generateTrajectory(
                          new Pose2d(0, 0, new Rotation2d(0)),
                          List.of(new Translation2d(-0.5, -1),new Translation2d(-1.0, -1.6),
                          new Translation2d(-1.58, -1.6)
                          ),
                          new Pose2d(-1.28,0,new Rotation2d(0)),
                        trajectoryConfig);


                var trajectorymidtopoint =
                        TrajectoryGenerator.generateTrajectory(
                          new Pose2d(0, 0, new Rotation2d(0)),
                          List.of(new Translation2d(0.5, 0),new Translation2d(1, 0.1)),
                          new Pose2d(0,0, new Rotation2d(0)),
                        trajectoryConfig);
        
                var goSecondNote =
                        TrajectoryGenerator.generateTrajectory(
                        new Pose2d(0, 0, new Rotation2d(0)),
                        List.of(new Translation2d(-1.3, 0)),
                        new Pose2d(-1.3,0, new Rotation2d(0)),
                      trajectoryConfig);
        
                var secondNoteToSpeaker =
                        TrajectoryGenerator.generateTrajectory(
                         new Pose2d(-1.3, 0, new Rotation2d(0)),
                         List.of(new Translation2d(-1.3, 0),new Translation2d(0,0)),
                         new Pose2d(0,0, new Rotation2d(0)),
                      trajectoryConfig);
                                 
                // return new AutoNoteShoot(m_shooter, m_feeder, () -> 55).withTimeout(2)
                // .andThen(pathCommand(trajectoryOne).alongWith(new pickupforAuto(m_intake, m_joystick).withTimeout(3.8))
                // .withTimeout(5)
                // .andThen(new InstantCommand(()-> m_intake.StopNoteMotor())
                // .andThen(new InstantCommand(()-> swerveSubsystem.stopModules()))))
                // .andThen(new fedleme(m_intake, m_feeder, m_joystick, m_shooter)).withTimeout(8.5)
                // .andThen(new InstantCommand(()-> m_intake.StopNoteMotor()))
                // .andThen(new AutoNoteShoot(m_shooter, m_feeder, ()-> 100))
                // .andThen(pathCommand(trajectoryTwo).alongWith(new pickupforAuto(m_intake, m_joystick).withTimeout(11.5))
                // .alongWith(new InstantCommand(()-> m_intake.StopNoteMotor())
                // .andThen(new InstantCommand(()-> m_shooter.throwStop()))
                // .andThen(new InstantCommand(()-> swerveSubsystem.stopModules()))))
                // .andThen(new fedleme(m_intake, m_feeder, m_joystick, m_shooter)).withTimeout(13)
                // .andThen(new InstantCommand(()-> m_intake.StopNoteMotor())).andThen(new AutoNoteShoot(m_shooter,m_feeder,() -> 100)).
                // withTimeout(14.9)
                // .andThen(new InstantCommand(()-> m_shooter.throwStop()));

                // return new AutoNoteShoot(m_shooter, m_feeder, () -> 50).withTimeout(2)
                // .andThen(pathCommand(trajectoryOne).alongWith(new pickupforAuto(m_intake, m_joystick).withTimeout(3.75))
                // .withTimeout(4.75)
                // .andThen(new InstantCommand(()-> m_intake.StopNoteMotor())
                // .andThen(new InstantCommand(()-> swerveSubsystem.stopModules()))))
                // .andThen(new otofeedleme(m_intake, m_feeder, m_joystick, m_shooter)).withTimeout(6.75)
                // .andThen(new InstantCommand(()-> m_intake.StopNoteMotor()))
                // .andThen(new AutoNoteShoot(m_shooter, m_feeder, ()-> 70)).withTimeout(8)
                // .andThen(pathCommand(trajectoryThree).alongWith(new pickupforAuto(m_intake, m_joystick).withTimeout(10))
                // .alongWith(new InstantCommand(()-> m_intake.StopNoteMotor())
                // .andThen(new InstantCommand(()-> m_shooter.throwStop()))
                // .andThen(new InstantCommand(()-> swerveSubsystem.stopModules())
                // .withTimeout(11.5)))).withTimeout(12.5)
                // .andThen(new otofeedleme(m_intake, m_feeder, m_joystick, m_shooter)).withTimeout(13.5)
                // .andThen(new AutoNoteShoot(m_shooter,m_feeder,() -> 70))
                // .withTimeout(14.5)
                // .andThen(new InstantCommand(()-> m_shooter.throwStop()))
                // .andThen(new InstantCommand(()-> m_intake.StopNoteMotor()));

               // ÜÇ NOTA
        //        return new AutoNoteShoot(m_shooter, m_feeder, () -> 64).withTimeout(2)
        //        .andThen(pathCommand(trajectoryOne).alongWith(new pickupforAuto(m_intake, m_joystick).withTimeout(3.75))
        //        .withTimeout(4.75)
        //        .andThen(new InstantCommand(()-> m_intake.StopNoteMotor())
        //        .andThen(new InstantCommand(()-> swerveSubsystem.stopModules()))))
        //        .andThen(new otofeedleme(m_intake, m_feeder, m_joystick, m_shooter)).withTimeout(6.75)
        //        .andThen(new InstantCommand(()-> m_intake.StopNoteMotor()))
        //        .andThen(new AutoNoteShoot(m_shooter, m_feeder, ()-> 70)).withTimeout(8)
        //        .andThen(pathCommand(note3path).alongWith(new pickupforAuto(m_intake, m_joystick).withTimeout(9.5))
        //        .alongWith(new InstantCommand(()-> m_intake.StopNoteMotor())
        //        .andThen(new InstantCommand(()-> m_shooter.throwStop()))
        //        .andThen(new InstantCommand(()-> swerveSubsystem.stopModules())
        //        .withTimeout(10.5)))).
        //        withTimeout(11)
        //        .andThen(new InstantCommand(()-> swerveSubsystem.stopModules()))
        //        .andThen(new otofeedleme(m_intake, m_feeder, m_joystick, m_shooter)).withTimeout(13.5)
        //        .andThen(new AutoNoteShoot(m_shooter,m_feeder,() -> 47))
        //        .withTimeout(14.6)
        //        .andThen(new InstantCommand(()-> m_shooter.throwStop()))
        //        .andThen(new InstantCommand(()-> m_intake.StopNoteMotor()));

        if(mode == 3){
                return new AutoNoteShoot(m_shooter, m_feeder, () -> 64).withTimeout(2)
                .andThen(pathCommand(trajectoryOne).alongWith(new pickupforAuto(m_intake, m_joystick).withTimeout(3.75))
                .withTimeout(4.75)
                .andThen(new InstantCommand(()-> m_intake.StopNoteMotor())
                .andThen(new InstantCommand(()-> swerveSubsystem.stopModules()))))
                .andThen(new otofeedleme(m_intake, m_feeder, m_joystick, m_shooter)).withTimeout(6.75)
                .andThen(new InstantCommand(()-> m_intake.StopNoteMotor()))
                .andThen(new AutoNoteShoot(m_shooter, m_feeder, ()-> 70)).withTimeout(8)
                .andThen(pathCommand(note3pathforblue).alongWith(new pickupforAuto(m_intake, m_joystick).withTimeout(9.5))
                .alongWith(new InstantCommand(()-> m_intake.StopNoteMotor())
                .andThen(new InstantCommand(()-> m_shooter.throwStop()))
                .andThen(new InstantCommand(()-> swerveSubsystem.stopModules())
                .withTimeout(10.5)))).
                withTimeout(11)
                .andThen(new InstantCommand(()-> swerveSubsystem.stopModules()))
                .andThen(new otofeedleme(m_intake, m_feeder, m_joystick, m_shooter)).withTimeout(13.5)
                .andThen(new AutoNoteShoot(m_shooter,m_feeder,() -> 47))
                .withTimeout(14.6)
                .andThen(new InstantCommand(()-> m_shooter.throwStop()))
                .andThen(new InstantCommand(()-> m_intake.StopNoteMotor()));
               }     
        
        else if(mode == 2){
                return new AutoNoteShoot(m_shooter, m_feeder, () -> 64).withTimeout(2)
                .andThen(pathCommand(trajectoryOne).alongWith(new pickupforAuto(m_intake, m_joystick).withTimeout(3.75))
                .withTimeout(4.75)
                .andThen(new InstantCommand(()-> m_intake.StopNoteMotor())
                .andThen(new InstantCommand(()-> swerveSubsystem.stopModules()))))
                .andThen(new otofeedleme(m_intake, m_feeder, m_joystick, m_shooter)).withTimeout(6.75)
                .andThen(new InstantCommand(()-> m_intake.StopNoteMotor()))
                .andThen(new AutoNoteShoot(m_shooter, m_feeder, ()-> 70)).withTimeout(8)
                .andThen(pathCommand(note3path).alongWith(new pickupforAuto(m_intake, m_joystick).withTimeout(9.5))
                .alongWith(new InstantCommand(()-> m_intake.StopNoteMotor())
                .andThen(new InstantCommand(()-> m_shooter.throwStop()))
                .andThen(new InstantCommand(()-> swerveSubsystem.stopModules())
                .withTimeout(10.5)))).
                withTimeout(11)
                .andThen(new InstantCommand(()-> swerveSubsystem.stopModules()))
                .andThen(new otofeedleme(m_intake, m_feeder, m_joystick, m_shooter)).withTimeout(13.5)
                .andThen(new AutoNoteShoot(m_shooter,m_feeder,() -> 47))
                .withTimeout(14.6)
                .andThen(new InstantCommand(()-> m_shooter.throwStop()))
                .andThen(new InstantCommand(()-> m_intake.StopNoteMotor()));
               }
               else if(mode == 1){

                return new AutoNoteShoot(m_shooter, m_feeder, () -> 64).withTimeout(2)
                .andThen(pathCommand(trajectoryOne).alongWith(new pickupforAuto(m_intake, m_joystick).withTimeout(3.75))
                .withTimeout(4.75)
                .andThen(new InstantCommand(()-> m_intake.StopNoteMotor())
                .andThen(new InstantCommand(()-> swerveSubsystem.stopModules()))))
                .andThen(new otofeedleme(m_intake, m_feeder, m_joystick, m_shooter)).withTimeout(6.75)
                .andThen(new InstantCommand(()-> m_intake.StopNoteMotor()))
                .andThen(new AutoNoteShoot(m_shooter, m_feeder, ()-> 73)).withTimeout(8);
               }
               else if(mode == 0){
                return new AutoNoteShoot(m_shooter, m_feeder, () -> 55).withTimeout(3)
                .andThen(pathCommand(trajectory5));
                 }
                 
               else{
                return new AutoNoteShoot(m_shooter, m_feeder, () -> 64).withTimeout(2)
                .andThen(pathCommand(trajectoryOne).alongWith(new pickupforAuto(m_intake, m_joystick).withTimeout(3.75))
                .withTimeout(4.75)
                .andThen(new InstantCommand(()-> m_intake.StopNoteMotor())
                .andThen(new InstantCommand(()-> swerveSubsystem.stopModules()))))
                .andThen(new otofeedleme(m_intake, m_feeder, m_joystick, m_shooter)).withTimeout(6.75)
                .andThen(new InstantCommand(()-> m_intake.StopNoteMotor()))
                .andThen(new AutoNoteShoot(m_shooter, m_feeder, ()-> 70)).withTimeout(8);
               }

                // İKİ NOTA

                // return new AutoNoteShoot(m_shooter, m_feeder, () -> 64).withTimeout(2)
                // .andThen(pathCommand(trajectoryOne).alongWith(new pickupforAuto(m_intake, m_joystick).withTimeout(3.75))
                // .withTimeout(4.75)
                // .andThen(new InstantCommand(()-> m_intake.StopNoteMotor())
                // .andThen(new InstantCommand(()-> swerveSubsystem.stopModules()))))
                // .andThen(new otofeedleme(m_intake, m_feeder, m_joystick, m_shooter)).withTimeout(6.75)
                // .andThen(new InstantCommand(()-> m_intake.StopNoteMotor()))
                // .andThen(new AutoNoteShoot(m_shooter, m_feeder, ()-> 70)).withTimeout(8);

                // İKİ NOTA ESKİ

                // return new AutoNoteShoot(m_shooter, m_feeder, () -> 60).withTimeout(2)
                // .andThen(pathCommand(trajectoryOne).alongWith(new pickupforAuto(m_intake, m_joystick).withTimeout(4))
                // .withTimeout(5)
                // .andThen(new InstantCommand(()-> m_intake.StopNoteMotor())
                // .andThen(new InstantCommand(()-> swerveSubsystem.stopModules()))))
                // .andThen(new InstantCommand(()-> m_joystick.intakemodchange(0)))
                // .andThen(new fedleme(m_intake, m_feeder, m_joystick, m_shooter)).withTimeout(8.5)
                // .andThen(new AutoNoteShoot(m_shooter, m_feeder, ()-> 60));

                // BİR NOTA

                // return new AutoNoteShoot(m_shooter, m_feeder, () -> 55).withTimeout(3);

                // sıfır nota (kullanmanız tavsiye olunmaz)

                // return null;


        
                }
        
        
                public SwerveControllerCommand pathCommand(Trajectory m_tra){
                        
                PIDController xController = new PIDController(0.08, 0, 0);
                PIDController yController = new PIDController(0.08, 0, 0);
                
                ProfiledPIDController thetaController = new ProfiledPIDController(
                        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
                thetaController.enableContinuousInput(-Math.PI, Math.PI);
        
                        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                        4,
                        3)
                                .setKinematics(DriveConstants.kDriveKinematics);
        
                SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                        m_tra,
                        swerveSubsystem::getPose,
                        DriveConstants.kDriveKinematics,
                        xController,
                        yController,
                        thetaController,
                        swerveSubsystem::OtosetModuleStates,
                        swerveSubsystem);
        
                return swerveControllerCommand;        
        
                }
}
