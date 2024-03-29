// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DoritoClimber;
import frc.robot.subsystems.LEDLights;

public class ClimberStageOneRelease extends CommandBase {

  private final DoritoClimber sysClimber;
  private final LEDLights sysLedLights;

  /** Creates a new ClimberStageOneRelease. */
  public ClimberStageOneRelease(DoritoClimber inSysClimber, LEDLights inSysLedLights)  {
    sysClimber = inSysClimber;
    sysLedLights = inSysLedLights;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(sysClimber);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    sysClimber.toggleClimberStageOne(true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  //  sysClimber.toggleClimberStageOne(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   if (sysClimber.getClimberStageOne() == true)
     return true;
   else return false;
    
  }
}
