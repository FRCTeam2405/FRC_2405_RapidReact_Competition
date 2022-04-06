// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.DoritoSpin;

public class DoritoMotor extends CommandBase {

  private final DoritoSpin sysClimber;

  /** Creates a new ClimberStage1. */
  public DoritoMotor(DoritoSpin inSysClimber)  {
    sysClimber = inSysClimber;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(sysClimber);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    sysClimber.DoritoClimberMotor(SmartDashboard.getNumber("doritoSpinner", ClimberConstants.DORITO_DEFAULT_SPEED));  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sysClimber.DoritoClimberMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
//    if (sysClimber.getMotorSpinValue() == SmartDashboard.getNumber("doritoSpinner", ClimberConstants.DORITO_DEFAULT_SPEED))
//      return true;
//    else
      return false;
  }
}
