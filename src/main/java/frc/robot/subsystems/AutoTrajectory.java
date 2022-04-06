// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.nio.file.Path;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainAutonomousConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.DrivetrainConstants.Autonomous;

public class AutoTrajectory extends SubsystemBase {
  // Autonomous Trajectory(s)
  
  // Basic One
  public Trajectory trajectoryBasicOneWaypoint01 = new Trajectory();

  // -- Left One
  public Trajectory trajectoryAutocCargoOneWaypoint01 = new Trajectory();
  public Trajectory trajectoryAutocCargoOneWaypoint02 = new Trajectory();
  public Trajectory trajectoryAutocCargoOneWaypoint03 = new Trajectory();

  // -- Left Two
  public Trajectory trajectoryLeftTwoWaypoint01 = new Trajectory();
  public Trajectory trajectoryLeftTwoWaypoint02 = new Trajectory();
  public Trajectory trajectoryLeftTwoWaypoint03 = new Trajectory();

  // -- Right One
  public Trajectory trajectoryRightOneWaypoint01 = new Trajectory();
  public Trajectory trajectoryRightOneWaypoint02 = new Trajectory();
  public Trajectory trajectoryRightOneWaypoint03 = new Trajectory();

  // -- Right Two  
  public Trajectory trajectoryRightTwoWaypoint01 = new Trajectory();
  public Trajectory trajectoryRightTwoWaypoint02 = new Trajectory();
  public Trajectory trajectoryRightTwoWaypoint03 = new Trajectory();
 
  
  /** Creates a new AutoTrajectory. */
  public AutoTrajectory() {

    // ----------------------------------
    // Initialize Autonomous Trajectories
    // ----------------------------------
    // -- Basic One 
    // -- -- sp: anywhere (will get out of zone from any angle)
    // -- -- Single Waypoint
    // -- -- Move off line from start
    // ----------------------------------
    
    // -- Basic One -- Waypoint 01
    try {
      Path autoPathBasicOneWaypoint01 = Filesystem.getDeployDirectory().toPath().resolve(Autonomous.TrajectoryPathWeaver.BasicOne.JsonPath.TRAJECTORY_PATH_JSON_BASIC_ONE_WP_01);
      trajectoryBasicOneWaypoint01 = TrajectoryUtil.fromPathweaverJson(autoPathBasicOneWaypoint01);

    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + Autonomous.TrajectoryPathWeaver.BasicOne.JsonPath.TRAJECTORY_PATH_JSON_BASIC_ONE_WP_01, ex.getStackTrace());
    }
    
    // --------------------------------
    // Initialize Autonomous Trajectory
    // --------------------------------
    // -- Left One (Left Center)
    // -- -- sp: Position on Left-most side of hub (center of left zone)
    // -- -- sp: Align rear of robot in line with first ball
    // -- -- Move to first ball behind robot
    // -- -- Move to collect ball 
    // -- -- Rotate to lineup high goal 
    // ----------------------------------

    // -- Left One -- Waypoint 01
    try {
      Path autoPathAutoCargoOneWaypoint01 = Filesystem.getDeployDirectory().toPath().resolve(Autonomous.TrajectoryPathWeaver.AutoCargo.JsonPath.TRAJECTORY_PATH_JSON_AUTO_CARGO_WP_01);
      trajectoryAutocCargoOneWaypoint01 = TrajectoryUtil.fromPathweaverJson(autoPathAutoCargoOneWaypoint01);

    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + Autonomous.TrajectoryPathWeaver.AutoCargo.JsonPath.TRAJECTORY_PATH_JSON_AUTO_CARGO_WP_01, ex.getStackTrace());
    }

    // -- Left One -- Waypoint 02
    try {
      java.nio.file.Path autoPathAutoCargoOneWaypoint02 = Filesystem.getDeployDirectory().toPath().resolve(Autonomous.TrajectoryPathWeaver.AutoCargo.JsonPath.TRAJECTORY_PATH_JSON_AUTO_CARGO_WP_02);
      trajectoryAutocCargoOneWaypoint02 = TrajectoryUtil.fromPathweaverJson(autoPathAutoCargoOneWaypoint02);

    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + Autonomous.TrajectoryPathWeaver.AutoCargo.JsonPath.TRAJECTORY_PATH_JSON_AUTO_CARGO_WP_02, ex.getStackTrace());
    }

    // -- Left One -- Waypoint 03
    try {
      java.nio.file.Path autoPathAutoCargoOneWaypoint03 = Filesystem.getDeployDirectory().toPath().resolve(Autonomous.TrajectoryPathWeaver.AutoCargo.JsonPath.TRAJECTORY_PATH_JSON_AUTO_CARGO_WP_03);
      trajectoryAutocCargoOneWaypoint03 = TrajectoryUtil.fromPathweaverJson(autoPathAutoCargoOneWaypoint03);

    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + Autonomous.TrajectoryPathWeaver.AutoCargo.JsonPath.TRAJECTORY_PATH_JSON_AUTO_CARGO_WP_03, ex.getStackTrace());
    }

    // --------------------------------
    // Initialize Autonomous Trajectory
    // --------------------------------
    // -- Left Two (Left Corner)
    // -- -- sp: Position on Left side of hub / leftmost spot on tarmac zone
    // -- -- sp: Align robot along left edge of hub
    // -- -- Move to first ball (left field) on opposite side
    // -- -- Move to collect ball 
    // -- -- Rotate to lineup high goal 
    // ----------------------------------

    // -- Left Two -- Waypoint 01
    try {
      Path autoPathLeftTwoWaypoint01 = Filesystem.getDeployDirectory().toPath().resolve(Autonomous.TrajectoryPathWeaver.LeftTwo.JsonPath.TRAJECTORY_PATH_JSON_LEFT_TWO_WP_01);
      trajectoryLeftTwoWaypoint01 = TrajectoryUtil.fromPathweaverJson(autoPathLeftTwoWaypoint01);

    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + Autonomous.TrajectoryPathWeaver.LeftTwo.JsonPath.TRAJECTORY_PATH_JSON_LEFT_TWO_WP_01, ex.getStackTrace());
    }

    // -- Left Two -- Waypoint 02
    try {
      java.nio.file.Path autoPathLeftTwoWaypoint02 = Filesystem.getDeployDirectory().toPath().resolve(Autonomous.TrajectoryPathWeaver.LeftTwo.JsonPath.TRAJECTORY_PATH_JSON_LEFT_TWO_WP_02);
      trajectoryLeftTwoWaypoint02 = TrajectoryUtil.fromPathweaverJson(autoPathLeftTwoWaypoint02);

    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + Autonomous.TrajectoryPathWeaver.LeftTwo.JsonPath.TRAJECTORY_PATH_JSON_LEFT_TWO_WP_02, ex.getStackTrace());
    }

    // -- Left Two -- Waypoint 03
    try {
      java.nio.file.Path autoPathLeftTwoWaypoint03 = Filesystem.getDeployDirectory().toPath().resolve(Autonomous.TrajectoryPathWeaver.LeftTwo.JsonPath.TRAJECTORY_PATH_JSON_LEFT_TWO_WP_03);
      trajectoryLeftTwoWaypoint03 = TrajectoryUtil.fromPathweaverJson(autoPathLeftTwoWaypoint03);

    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + Autonomous.TrajectoryPathWeaver.LeftTwo.JsonPath.TRAJECTORY_PATH_JSON_LEFT_TWO_WP_03, ex.getStackTrace());
    }


//    Trajectory testBasic01 = genAutonomousTrajectory(inPoseStart, inWaypointList, inPoseEnd, inTrajectoryConfig)


  }


  // Autonomous Trajectory Method(s)
  // -------------------------------

  // --------------------------------------
  // Autonomous Configuration / Generation
  // --------------------------------------

  // Set Autonomous Pose
  public Pose2d genRobotPose(double inPoseX, double inPoseY, double inRotation) {
    
    Pose2d outputPose = new Pose2d(inPoseX, inPoseY, new Rotation2d(inRotation));
    return outputPose;
  }

  public List<Translation2d> genWaypointList(List<Translation2d> inWaypointList) {

    List<Translation2d> outputWaypointList = inWaypointList;
    return outputWaypointList;
  }

  // Set Autonomous Trajectory
  public Trajectory genAutonomousTrajectory(Pose2d inPoseStart, List<Translation2d> inWaypointList, Pose2d inPoseEnd, TrajectoryConfig inTrajectoryConfig) {

    inTrajectoryConfig = this.genAutonomousTrajectoryConfig();
    inPoseStart = this.genRobotPose(0, 0, 0);
    
    Trajectory outputTrajectory =
    TrajectoryGenerator.generateTrajectory(
      // Start at the origin facing the +X direction
      inPoseStart, 
      // Pass through these two interior waypoints, making an 's' curve path
      inWaypointList, 
      // End 3 meters straight ahead of where we started, facing forward
      inPoseEnd, 
      inTrajectoryConfig);

      return outputTrajectory;
  }

  // Set Autonomous Trajectory Config
  public TrajectoryConfig genAutonomousTrajectoryConfig() {

    // Trajectory
    TrajectoryConfig outputTrajectoryConfig = new TrajectoryConfig(
      DrivetrainConstants.Autonomous.DRIVETRAIN_MAX_SPEED_PER_SECOND_METERS, 
        DrivetrainConstants.Autonomous.DRIVETRAIN_MAX_ACCELERATION_PER_SECOND_SQUARED_METERS
        ).setKinematics(DrivetrainAutonomousConstants.DRIVETRAIN_MECANUM_KINEMATICS);

    return outputTrajectoryConfig;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}