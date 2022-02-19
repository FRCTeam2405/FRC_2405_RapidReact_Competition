// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ShootLow extends CommandBase {
  /** Creates a new ShootLow. */
  public ShootLow() {
    // Use addRequirements() here to declare subsystem dependencies.
  addRequirements(RobotContainer.m_Shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     //Defining shootLow 
   boolean shootLow = RobotContainer.driverController.getAButton();
  
   if (shootLow){
    //Setting percent output if A pressed 
   RobotContainer.m_Shooter.Shoot(ControlMode.PercentOutput, .20);
       
     } else{
      RobotContainer.m_Shooter.Shoot(ControlMode.PercentOutput, 0);
     }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_Shooter.Shoot(ControlMode.PercentOutput, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
