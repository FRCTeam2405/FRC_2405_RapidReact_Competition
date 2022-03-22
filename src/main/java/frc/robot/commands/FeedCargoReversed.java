// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.FeederConstants;
import frc.robot.subsystems.Feeder;

public class FeedCargoReversed extends CommandBase {

  private final Feeder sysFeeder;
  
  /** Creates a new feedCargo. */
  public FeedCargoReversed(Feeder inSysFeeder) {
    sysFeeder = inSysFeeder;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(sysFeeder);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double feederPercentCargo = SmartDashboard.getNumber("FeederPercentOutputReversed", FeederConstants.FEEDER_DEFAULT_OUTPUT_REVERSED);  
    sysFeeder.feedCargo(feederPercentCargo);

    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sysFeeder.feedCargo(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return false;
    }
  }