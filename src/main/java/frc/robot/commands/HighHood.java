// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;

public class HighHood extends CommandBase {
  public final Shooter sysShooter;
  /** Creates a new hood. */

  public HighHood(Shooter inSysShooter) {
    sysShooter = inSysShooter;

    // Use addRequirements() here to declare subsystem dependencies.
  addRequirements(sysShooter);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
 sysShooter.Hood(-1 * SmartDashboard.getNumber("ShooterHood", ShooterConstants.SHOOTER_HOOD_DEFAULT_OUTPUT));
  
 SmartDashboard.putString("Hood State", "High");
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sysShooter.Hood(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
