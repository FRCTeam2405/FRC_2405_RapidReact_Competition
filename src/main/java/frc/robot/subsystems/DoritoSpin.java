// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.EncoderConstants.Dorito;

public class DoritoSpin extends SubsystemBase {
  //defining encoder
  RelativeEncoder doritoSpinnerEncoder = null;

  //Defining SparkMax
  CANSparkMax mainDoritoSparkMax = null; 

  public double encoderHomePos;
  
  /** Creates a new DoritoSpin. */
  public DoritoSpin() {
    //Defining SparkMax
    mainDoritoSparkMax = new CANSparkMax(ClimberConstants.MAIN_DORITO_SPARKMAX, MotorType.kBrushless);

    doritoSpinnerEncoder = mainDoritoSparkMax.getEncoder();

    encoderHomePos = this.getDoritoEncoderPos();

    //Initalizing Dorito Spinner Encoders
    // doritoSpinnerEncoder = new Encoder(
    // Dorito.DORITOMOTOR_ENCODER_CHANNEL_01,
    // Dorito.DORITOMOTOR_ENCODER_CHANNEL_02,
    // Dorito.DORITO_MOTOR_ENCODER_ISREVERSED
    // );

    

  }

  public void DoritoClimberMotor(double inputAmount) {
    mainDoritoSparkMax.set(inputAmount);
  }

  public double getMotorSpinValue() {
    return mainDoritoSparkMax.get();
  }

  public double getDoritoEncoderPos() {
    return doritoSpinnerEncoder.getPosition();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Dorito Encoder Position", this.getDoritoEncoderPos());
  }
}
