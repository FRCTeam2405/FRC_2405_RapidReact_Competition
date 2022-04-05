// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.LEDLights;
import frc.robot.subsystems.Shooter;

public class ShootLow extends CommandBase {
  
  private final Shooter sysShooter;
  private final LEDLights sysLedLights;

  /** Creates a new ShootLow. */
  public ShootLow(Shooter inSysShooter, LEDLights inSysLedLights) {
    sysShooter = inSysShooter;
    sysLedLights = inSysLedLights;

    // Use addRequirements() here to declare subsystem dependencies.
  addRequirements(sysShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     //Defining shootLow 
   //boolean shootLow = RobotContainer.driverController.getAButton();
  
   //if (shootLow){
    //Setting percent output if A pressed 
   // sysShooter.Shoot(ControlMode.PercentOutput, .20);
       
    // } else{
    //  sysShooter.Shoot(ControlMode.PercentOutput, 0);
   //  }

   double feederPercentCargo = SmartDashboard.getNumber("ShootLowPercentOutput", ShooterConstants.SHOOTLOW_DEFAULT_OUTPUT);  
    sysShooter.Shoot(ControlMode.PercentOutput, feederPercentCargo);
    sysLedLights.setLEDValue(LEDConstants.LED_SETTING_SHOOT_LOW);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_Shooter.Shoot(ControlMode.PercentOutput, 0);
    sysLedLights.setLEDValue(LEDConstants.LED_SETTING_DEFAULT);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   // if (sysShooter.getShootLowValue() == SmartDashboard.getNumber("ShootLowPercentOutput", ShooterConstants.SHOOTLOW_DEFAULT_OUTPUT))
    //  return true; 
   // else
      return false;
  }
}
