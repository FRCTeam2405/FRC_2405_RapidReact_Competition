// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DoritoClimber;

public class ClimberSolenoid extends CommandBase {
  private final DoritoClimber sysDoritoClimber;
  /** Creates a new ClimberSolenoidOne. */
private final int sulID;
private final boolean sulValue; 
  public ClimberSolenoid(DoritoClimber inSysDoritoClimber, int inSulID, boolean inSulValue) {

    // Use addRequirements() here to declare subsystem dependencies.
    sysDoritoClimber = inSysDoritoClimber;
    sulID = inSulID;
    sulValue = inSulValue;
    addRequirements(inSysDoritoClimber);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (sulID == 1){
      sysDoritoClimber.toggleClimberOne(true);
    }
    else if (sulID == 2){
      sysDoritoClimber.toggleClimberTwo(true);
    }
    else if (sulID == 3){
      sysDoritoClimber.toggleClimberThree(true);
    }

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
