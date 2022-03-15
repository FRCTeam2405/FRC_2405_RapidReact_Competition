// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.IntakeConstants;

public class DoritoClimber extends SubsystemBase {
  /** Creates a new DoritoClimber. */
  //Defining Solenoids
  Solenoid doritoSolenoidOne = null;
  Solenoid doritoSolenoidTwo = null;
  Solenoid doritoSolenoidThree = null;
  Solenoid doritoLifterSolenoid = null;

  //defining encoder
   Encoder doritoSpinnerEncoder = null;

  //Defining SparkMax
  CANSparkMax mainDoritoSparkMax = null; 

  public DoritoClimber() {
    //Defining Solenoids
    doritoSolenoidOne = new Solenoid(IntakeConstants.PORT_PCM_MAIN, PneumaticsModuleType.CTREPCM, ClimberConstants.DORITO_SOLENOID_PORT_ONE);
    doritoSolenoidTwo = new Solenoid(IntakeConstants.PORT_PCM_MAIN, PneumaticsModuleType.CTREPCM, ClimberConstants.DORITO_SOLENOID_PORT_TWO);
    doritoSolenoidThree = new Solenoid(IntakeConstants.PORT_PCM_MAIN, PneumaticsModuleType.CTREPCM, ClimberConstants.DORITO_SOLENOID_PORT_THREE);
    doritoLifterSolenoid = new Solenoid(IntakeConstants.PORT_PCM_MAIN, PneumaticsModuleType.CTREPCM, ClimberConstants.DORITO_LIFTER_SOLENOID_PORT);

    //Defining SparkMax
    mainDoritoSparkMax = new CANSparkMax(ClimberConstants.MAIN_DORITO_SPARKMAX, MotorType.kBrushless);
/*
  //Initalizing Dorito Spinner Encoders
    doritoSpinnerEncoder = new Encoder(
    Dorito.DORITOMOTOR_ENCODER_CHANNEL_01,
    Dorito.DORITOMOTOR_ENCODER_CHANNEL_02,
    Dorito.DORITO_MOTOR_ENCODER_ISREVERSED
);
*/
  }

  public void toggleClimberStageOne(boolean climberOneActive) {
    doritoSolenoidOne.set(climberOneActive);
  }
  public void toggleClimberTwo(boolean climberTwoActive) {
    doritoSolenoidTwo.set(climberTwoActive);
  }
  public void toggleClimberThree(boolean climberThreeActive) {
    doritoSolenoidThree.set(climberThreeActive);
  }

  public void DoritoClimberMotor(double inputAmount) {
    mainDoritoSparkMax.set(inputAmount);
  }

  public void toggleDoritoLifterSolenoid(boolean isEngaged) {
    doritoLifterSolenoid.set(isEngaged);
  }

  public boolean getClimberStageOne() {
    return doritoSolenoidOne.get();
  }
  public boolean getClimberTwo() {
    return doritoSolenoidTwo.get();
  }
  public boolean getClimberThree() {
    return doritoSolenoidThree.get();
  }
  public boolean getDoritoLifterSulenoid() {
    return doritoLifterSolenoid.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
