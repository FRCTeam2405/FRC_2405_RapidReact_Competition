// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;

public class Feeder extends SubsystemBase {
  /** Creates a new feederMotor. */
  CANSparkMax mainfeederMotor = null;

  DigitalInput bottomFeederLimit = null;

  DigitalInput topFeederLimit = null;

  public Feeder() {

mainfeederMotor = new CANSparkMax(FeederConstants.MAIN_FEEDER_MOTOR, MotorType.kBrushless);

bottomFeederLimit = new DigitalInput(0);
topFeederLimit = new DigitalInput(1);

}

public boolean LimitSwitchTripped() {
  if (bottomFeederLimit.get() == false && topFeederLimit.get() == false) {
    return false;
  }
  else {
    return true;
  }
}

public void feedCargo(double inputAmount) {
mainfeederMotor.set(inputAmount);
}

public double getFeedCargoValue() {
  return mainfeederMotor.get();
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("LimitSwitchTripped", this.LimitSwitchTripped());
  }
}
