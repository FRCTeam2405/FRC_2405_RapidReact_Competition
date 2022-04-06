// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.DriveTrain;

public class CartesianDriveForAuton extends CommandBase {
  private double waitTimeStart = 0;
  private double waitTimeSetting = 0;
  private double waitTimeElapsed = 0;

  private double moveSpeedY = 0;
  private double moveSpeedX = 0;
  private double rotationSpeedZ = 0;

  private final DriveTrain sysDriveTrain;

  /** Creates a new CartesianDriveForAuton. */
  public CartesianDriveForAuton(DriveTrain inSysDriveTrain, double inMoveSpeedY, double inMoveSpeedX, double inRotationSpeedZ, double inWaitTimeSetting) {
    // Use addRequirements() here to declare subsystem dependencies.
    sysDriveTrain = inSysDriveTrain;
    waitTimeSetting = inWaitTimeSetting;
    moveSpeedY = inMoveSpeedY;
    moveSpeedX = inMoveSpeedX;
    rotationSpeedZ = inRotationSpeedZ;

    addRequirements(sysDriveTrain);


  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
 
    // Get Start time when command in scheduled
    waitTimeStart = Timer.getFPGATimestamp();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //sysDriveTrain.cartesianDrive(SmartDashboard.getNumber("Auton Time", DrivetrainConstants.VELOCITY_DRIVING_IN_AUTON), 0, 0, false);
    
    sysDriveTrain.cartesianDrive( moveSpeedY, moveSpeedX, rotationSpeedZ, false);

    // Get Time Elapsed since Start
    waitTimeElapsed = Timer.getFPGATimestamp() - waitTimeStart;

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    // Get Time Elapsed since Start
    waitTimeElapsed = Timer.getFPGATimestamp() - waitTimeStart;
    
    // Finish command when Time Elapsed is >= Time Set
    if (waitTimeElapsed >= waitTimeSetting) {
      return true;
    }
    else {
      return false;
    }
  }
}
