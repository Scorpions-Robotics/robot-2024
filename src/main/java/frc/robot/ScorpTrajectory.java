/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;
//import edu.wpi.first.math.proto.Trajectory;
import edu.wpi.first.math.trajectory.*;;

public class ScorpTrajectory {

  public Trajectory[] FirstTrajectory = new Trajectory[1];

    private SwerveSubsystem swerveSubsystem;
 
    public ScorpTrajectory(SwerveSubsystem swerveSubsystem){
                
         this.swerveSubsystem = swerveSubsystem;
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(4,3).setKinematics(DriveConstants.kDriveKinematics);

     PIDController xController = new PIDController(0.08, 0, 0);
     PIDController yController = new PIDController(0.08, 0, 0);
    
     ProfiledPIDController thetaController = new ProfiledPIDController(
     AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        
     thetaController.enableContinuousInput(-Math.PI, Math.PI);


     //trajectoryyyy
      FirstTrajectory[0] = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(new Translation2d(-1.28, 0),new Translation2d(-0.56, -0.91)),
            new Pose2d(-1.28, -1.44, new Rotation2d(0)),
            trajectoryConfig);


    }
    public SwerveControllerCommand getController(Trajectory trajectory,PIDController xController,PIDController yController, ProfiledPIDController thetaController){

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
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