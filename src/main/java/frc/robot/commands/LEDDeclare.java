// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.LEDLights;

public class LEDDeclare extends CommandBase {
  private final LEDLights sysLEDLights;
  private final double LEDsetting;

  /** Creates a new LEDDeclare. */
  public LEDDeclare(LEDLights insysLEDLights, double inLEDsetting) {

    sysLEDLights = insysLEDLights;
    LEDsetting = inLEDsetting;

    // Use addRequirements() here to declare subsystem dependencies.  
    addRequirements(sysLEDLights);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    sysLEDLights.setLEDValue(SmartDashboard.getNumber("LEDSet", LEDConstants.LED_SETTING_DEFAULT));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sysLEDLights.setLEDValue(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
