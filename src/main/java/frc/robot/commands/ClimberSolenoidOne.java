// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DoritoClimber;

public class ClimberSolenoidOne extends CommandBase {
  private final DoritoClimber sysDoritoClimber;
  /** Creates a new ClimberSolenoidOne. */

  public ClimberSolenoidOne(DoritoClimber inSysDoritoClimber) {

    // Use addRequirements() here to declare subsystem dependencies.
    sysDoritoClimber = inSysDoritoClimber;
    addRequirements(inSysDoritoClimber);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    sysDoritoClimber.toggleClimberOne(true);
    sysDoritoClimber.toggleClimberTwo(true);
    sysDoritoClimber.toggleClimberThree(true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sysDoritoClimber.toggleClimberOne(false);
    sysDoritoClimber.toggleClimberTwo(false);
    sysDoritoClimber.toggleClimberThree(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
