// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.LEDLights;
import frc.robot.subsystems.Shooter;

public class ShooterOff extends CommandBase {
  /** Creates a new ShooterOff. */

  private final Shooter sysShooter;
  private final LEDLights sysLedLights;
  
  public ShooterOff(Shooter inSysShooter, LEDLights inSyLedLights) {
    sysShooter = inSysShooter;
    sysLedLights = inSyLedLights;
 
    addRequirements(sysShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    sysShooter.Shoot(ControlMode.PercentOutput, 0);
    sysLedLights.setLEDValue(LEDConstants.LED_SETTING_DEFAULT);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
