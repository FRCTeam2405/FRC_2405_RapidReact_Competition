// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class ShootHigh extends CommandBase {

  private final Shooter sysShooter;
  /** Creates a new ShootHigh. */
  public ShootHigh(Shooter inSysShooter) {
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
     //Defining shootHigh 

     
   // if (shootHigh){
    //Setting percent output if Y pressed 
   // RobotContainer.m_Shooter.Shoot(ControlMode.PercentOutput, .25);
      // .35 for high-shoot up close
      // .25 for low shot from one robot behind
       
    // } else{
     // RobotContainer.m_Shooter.Shoot(ControlMode.PercentOutput, 0);
    // }
    
    double feederPercentCargo = SmartDashboard.getNumber("ShooterPercentOutput", 0.35);  
    sysShooter.Shoot(ControlMode.PercentOutput, feederPercentCargo);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sysShooter.Shoot(ControlMode.PercentOutput, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
