// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.DriveTrain;

public class DriveMecanum extends CommandBase {
  private final DriveTrain sysDriveTrain;
  /** Creates a new DriveMecanam. */
  public DriveMecanum(DriveTrain inSysDriveTrain) {

    sysDriveTrain = inSysDriveTrain;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(sysDriveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double moveSpeedX = RobotContainer.driverController.getRightX();
    double rotateSpeedZ = RobotContainer.driverController.getLeftX();
    double moveSpeedY = RobotContainer.driverController.getLeftY();
    boolean gyroIsUsed = false;

  //Controllor Deadband
  if (moveSpeedY <= ControllerConstants.DRIVER_CONTROLLER_DEADBAND && moveSpeedY >= (-1 * ControllerConstants.DRIVER_CONTROLLER_DEADBAND)) {
  moveSpeedY = 0;
}
  if (moveSpeedX <= ControllerConstants.DRIVER_CONTROLLER_DEADBAND && moveSpeedX >= (-1 * ControllerConstants.DRIVER_CONTROLLER_DEADBAND)) {
  moveSpeedX = 0;
}
  if (rotateSpeedZ <= ControllerConstants.DRIVER_CONTROLLER_DEADBAND && rotateSpeedZ >= (-1 * ControllerConstants.DRIVER_CONTROLLER_DEADBAND)) {
  rotateSpeedZ = 0;
}  

 // Override moveSpeedX [strafe] - Left
 if (RobotContainer.driverController.getLeftTriggerAxis() != 0) {
  moveSpeedX = -1 * RobotContainer.driverController.getLeftTriggerAxis();
}

// Override moveSpeedX [strafe] - Right
if (RobotContainer.driverController.getRightTriggerAxis() != 0) {
  moveSpeedX = RobotContainer.driverController.getRightTriggerAxis();
}

  //Call cartesianDrive
  sysDriveTrain.cartesianDrive(moveSpeedY, moveSpeedX, rotateSpeedZ, gyroIsUsed);

}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sysDriveTrain.cartesianDrive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
