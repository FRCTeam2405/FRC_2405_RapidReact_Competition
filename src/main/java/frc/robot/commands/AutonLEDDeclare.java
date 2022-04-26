// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.concurrent.BlockingQueue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.LEDLights;

public class AutonLEDDeclare extends CommandBase {
  private final LEDLights sysLEDLights;
  private final double LEDsettingOne;
  private final double LEDsettingTwo;

  private double waitTimeStart = 0;
  private double waitTimeSetting = 0;
  private double waitTimeElapsed = 0;

  /** Creates a new LEDDeclare. */
  public AutonLEDDeclare(LEDLights insysLEDLights, double inLEDsettingOne, double inLEDsettingTwo, double inWaitTimeSetting) {
    waitTimeSetting = inWaitTimeSetting;

    sysLEDLights = insysLEDLights;
    LEDsettingOne = inLEDsettingOne;
    LEDsettingTwo = inLEDsettingTwo;

    // Use addRequirements() here to declare subsystem dependencies.  
    addRequirements(sysLEDLights);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    waitTimeStart = Timer.getFPGATimestamp();
  
    sysLEDLights.setLEDValue(LEDsettingTwo);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    waitTimeElapsed = Timer.getFPGATimestamp() - waitTimeStart;

    if (waitTimeElapsed >= waitTimeSetting) {
      sysLEDLights.setLEDValue(LEDsettingOne);
    }
    else if (waitTimeElapsed >= waitTimeSetting * 2) {
      sysLEDLights.setLEDValue(LEDsettingTwo);
      waitTimeStart = Timer.getFPGATimestamp();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
