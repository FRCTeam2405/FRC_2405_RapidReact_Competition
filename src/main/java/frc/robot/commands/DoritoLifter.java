// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DoritoClimber;

public class DoritoLifter extends CommandBase {

  private final DoritoClimber sysClimber;

  /** Creates a new ClimberStage1. */
  public DoritoLifter(DoritoClimber inSysClimber)  {
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
    sysClimber.toggleDoritoLifterSolenoid(true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sysClimber.toggleDoritoLifterSolenoid(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}