// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.DrivetrainAutonomousConstants;
import frc.robot.Constants.EncoderConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoSeqTest extends SequentialCommandGroup {
  /** Creates a new AutoSeqTest. */
  public AutoSeqTest(DriveTrain m_DriveTrain, Intake m_Intake, Feeder m_feedermotor, Shooter m_Shooter)  {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

     // Trajectory
   TrajectoryConfig drivetrainTrajectoryConfig = 
   new TrajectoryConfig(
     DrivetrainAutonomousConstants.DRIVETRAIN_MAX_SPEED_PER_SECOND_METERS, 
     DrivetrainAutonomousConstants.DRIVETRAIN_MAX_ACCELERATION_PER_SECOND_SQUARED_METERS
     ).setKinematics(DrivetrainAutonomousConstants.DRIVETRAIN_MECANUM_KINEMATICS);


    Trajectory exampleTrajectory = 
   TrajectoryGenerator.generateTrajectory(
     // Start at the origin facing the +X direction
     new Pose2d(0, 0, new Rotation2d(0)), 
     // Pass through these two interior waypoints, making an 's' curve path
     // test-to-try: List.of(new Translation2d(4, 0), new Translation2d(1, 1), new Translation2d(2, -1), new Translation2d(4, 0)), 
     List.of(new Translation2d(4, 0), 
     new Translation2d(1, 1), 
     new Translation2d(2, -1), 
     new Translation2d(4, 0)),
     // End 3 meters straight ahead of where we started, facing forward
     new Pose2d(3, 0, new Rotation2d(0)), 
     drivetrainTrajectoryConfig);


    addCommands(
      new MecanumControllerCommand(
        exampleTrajectory, 
        RobotContainer.m_DriveTrain::getPose, 
        EncoderConstants.DRIVETRAIN_ENCODER_FEED_FORWARD, 
        DrivetrainAutonomousConstants.DRIVETRAIN_MECANUM_KINEMATICS, 
        
        // Position contollers
        new PIDController(DrivetrainAutonomousConstants.DRIVETRAIN_CONTROLLER_PX, 0, 0), 
        new PIDController(DrivetrainAutonomousConstants.DRIVETRAIN_CONTROLLER_PY, 0, 0), 
        new ProfiledPIDController(DrivetrainAutonomousConstants.DRIVETRAIN_CONTROLLER_PTHETA, 0, 0, DrivetrainAutonomousConstants.DRIVETRAIN_THETA_CONTROLLER_CONSTRAINTS), 
        
        // Needed for normalizing wheel speeds
        DrivetrainAutonomousConstants.DRIVETRAIN_MAX_SPEED_PER_SECOND_METERS, 
        
        // Velocity PID's
        new PIDController(EncoderConstants.DRIVETRAIN_ENCODER_FRONT_LEFT_VELOCITY_PERCENT, 0, 0),
        new PIDController(EncoderConstants.DRIVETRAIN_ENCODER_BACK_LEFT_VELOCITY_PERCENT, 0, 0),
        new PIDController(EncoderConstants.DRIVETRAIN_ENCODER_FRONT_RIGHT_VELOCITY_PERCENT, 0, 0),
        new PIDController(EncoderConstants.DRIVETRAIN_ENCODER_BACK_RIGHT_VELOCITY_PERCENT, 0, 0),
   
        RobotContainer.m_DriveTrain::getCurrentDrivetrainWheelSpeeds, 
        RobotContainer.m_DriveTrain::setDriveMotorControllersVolts, 
        RobotContainer.m_DriveTrain

      ),

      //Drive

      //ShootHigh
      new ShootHigh(m_Shooter),

      //Intake Deploy
      new IntakeDeploy(m_Intake)
      
    );
   


   

  // Reset odometry to the starting pose of the trajectory.
  RobotContainer.m_DriveTrain.resetOdometry(exampleTrajectory.getInitialPose());

  // Run path following command, then stop at the end.
  // test-to-try: return drivetrainMecanumControllerCommand.andThen(() -> sysDriveTrain.cartesianDrive(0, 0, 0, false)).andThen(drivetrainMecanumControllerCommand.andThen(() -> sysDriveTrain.cartesianDrive(0, 0, 0, false)));
  //return drivetrainMecanumControllerCommand.andThen(() -> RobotContainer.m_DriveTrain.cartesianDrive(0, 0, 0, false));
  }
}
