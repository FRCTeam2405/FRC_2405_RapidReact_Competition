// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DriveMecanum extends CommandBase {
  /** Creates a new DriveMecanam. */
  public DriveMecanum() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_DriveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double moveSpeedX = RobotContainer.driverController.getLeftX();
    double rotateSpeedZ = RobotContainer.driverController.getRightX();
    double moveSpeedY = RobotContainer.driverController.getLeftY();
    boolean gyroIsUsed = false;

  //Call cartesianDrive
  RobotContainer.m_DriveTrain.cartesianDrive(moveSpeedY, moveSpeedX, rotateSpeedZ, gyroIsUsed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_DriveTrain.cartesianDrive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
