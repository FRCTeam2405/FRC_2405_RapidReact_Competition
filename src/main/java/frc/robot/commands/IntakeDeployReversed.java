// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class IntakeDeployReversed extends CommandBase {

  private final Intake sysIntake;
  
  /** Creates a new IntakeDeploy. */
  public IntakeDeployReversed(Intake inSysIntake) {
    sysIntake = inSysIntake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(sysIntake);
  }

  // Called when the command is initially scheduled.
  @Override 
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    sysIntake.ToggleIntake(SmartDashboard.getNumber("IntakePercentOutputReverced", IntakeConstants.INTAKE_DEFAULT_SPEED_REVERSED), true);
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sysIntake.ToggleIntake(0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //if (sysIntake.getIntakeMotorValue() == SmartDashboard.getNumber("IntakePercentOutputReverced", IntakeConstants.INTAKE_DEFAULT_SPEED_REVERSED))
    //  return true;
    //else
      return false;
  }
}
