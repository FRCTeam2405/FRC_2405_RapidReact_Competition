// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class DoritoClimber extends SubsystemBase {
  /** Creates a new DoritoClimber. */
  Solenoid doritoSolenoidOne = null;
  Solenoid doritoSulenoidTwo = null;
  Solenoid doritoSolenoidThree = null;

  public DoritoClimber() {
    doritoSolenoidOne = new Solenoid(PneumaticsModuleType.REVPH, ClimberConstants.DORITO_SOLENOID_ONE);
    doritoSulenoidTwo = new Solenoid(PneumaticsModuleType.REVPH, ClimberConstants.DORITO_SOLENOID_TWO);
    doritoSolenoidThree = new Solenoid(PneumaticsModuleType.REVPH, ClimberConstants.DORITO_SOLENOID_THREE);

  }

  public void toggleClimberOne(boolean climberOneActive) {
    doritoSolenoidOne.set(climberOneActive);
  }
  public void toggleClimberTwo(boolean climberTwoActive) {
    doritoSulenoidTwo.set(climberTwoActive);
  }
  public void toggleClimberThree(boolean climberThreeActive) {
    doritoSolenoidThree.set(climberThreeActive);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
