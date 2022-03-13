// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
 //Defining SparkMax
 CANSparkMax intakeMainSparkMax = null;
 
 //Defining Solenoid
 Solenoid intakeMainSolenoids = null;

  /** Creates a new Intake. */
  public Intake() {
   //Intializing SparkMax
   intakeMainSparkMax = new CANSparkMax(IntakeConstants.INTAKE_MAIN_SPARKMAX, MotorType.kBrushless);

   //Intake Reversal
   intakeMainSparkMax.setInverted(true);

   //Intializing Solenoid
   intakeMainSolenoids = new Solenoid(IntakeConstants.PORT_PCM_MAIN, PneumaticsModuleType.REVPH, IntakeConstants.INTAKE_SOLENOIDS_PORT);

  }

  public void ToggleIntake(double outPutValue, boolean intakeActive){
   intakeMainSparkMax.set(outPutValue);
    
   //Turning Solenoid on/off
   intakeMainSolenoids.set(intakeActive); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}