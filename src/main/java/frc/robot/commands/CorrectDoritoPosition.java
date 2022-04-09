// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.opencv.core.TickMeter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DoritoSpin;

public class CorrectDoritoPosition extends CommandBase {

  private final DoritoSpin sysDoritoSpin;
  /** Creates a new CorrectDoritoPosition. */
  public CorrectDoritoPosition(DoritoSpin inSysDoritoSpin)  {
    sysDoritoSpin = inSysDoritoSpin;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(sysDoritoSpin);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
