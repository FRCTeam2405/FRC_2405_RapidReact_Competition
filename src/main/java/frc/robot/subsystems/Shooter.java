// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  //Defining Main Shooter 
 WPI_TalonFX shooterMainTalonFX = null;
 //DigitalInput shooterLimitSwitchHigh = null;
 //DigitalInput shooterLimitSwitchLow = null;
 //

 CANSparkMax hoodMotor = null;

  public Shooter() {
    //Initalizing Main Shooter Motor
  shooterMainTalonFX = new WPI_TalonFX(ShooterConstants.SHOOTER_MAIN_TALONFX);
  hoodMotor = new CANSparkMax(ShooterConstants.SHOOTER_HOOD_TALONSRX, MotorType.kBrushed);
  //shooterLimitSwitchHigh = new DigitalInput(ShooterConstants.SHOOT_HIGH_HOOD_LIMIT);
  //shooterLimitSwitchLow = new DigitalInput(ShooterConstants.SHOOT_LOW_HOOD_LIMIT);

  
}
  
  
  public void Shoot(ControlMode motorMode, double outPutValue){
    shooterMainTalonFX.set(motorMode, outPutValue);
}

  public void Hood(double outPutValue){
    hoodMotor.set(outPutValue);
  }

  public double getShootMotorValue() {
    return shooterMainTalonFX.getSelectedSensorVelocity();
  }

  public double getShootLowValue() {
    return shooterMainTalonFX.get();
  }

  public double getShootHightValue() {
    return shooterMainTalonFX.get();
  }

  //public boolean LimitSwitchHigh(){
 //   return shooterLimitSwitchHigh.get();
//}

  //public boolean LimitSwitchLow(){
    //return shooterLimitSwitchLow.get();
//}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  SmartDashboard.putNumber("ShooterPercentOutput", shooterMainTalonFX.get());

  }
}
