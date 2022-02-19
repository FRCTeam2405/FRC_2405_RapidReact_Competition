// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  //Defining Main Shooter 
 WPI_TalonFX shooterMainTalonFX = null;
 
  public Shooter() {
    //Initalizing Main Shooter Motor
  shooterMainTalonFX = new WPI_TalonFX(ShooterConstants.SHOOTER_MAIN_TALONFX);
  
  
  }
  
  
  public void Shoot(ControlMode moterMode, double outPutValue){
  shooterMainTalonFX.set(moterMode, -1 * outPutValue);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  SmartDashboard.putNumber("Shooter Velocity", shooterMainTalonFX.getSelectedSensorVelocity());

  }
}
