// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainAutonomousConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.EncoderConstants;

public class DriveTrain extends SubsystemBase {
   /** Creates a new DriveTrain. */
  // Defining mecanumDrive
   MecanumDrive mecanumDrive = null;

   //Defining Motors
   WPI_TalonFX leftFrontTalonFX = null;
   WPI_TalonFX rightFrontTalonFX = null;
   WPI_TalonFX leftBackTalonFX = null;
   WPI_TalonFX rightBackTalonFX = null;

   //Defining Drivetrain Encoders
   Encoder drivetrainLeftFrontEncoder = null;
   Encoder drivetrainLeftBackEncoder = null;
   Encoder drivetrainRightFrontEncoder = null;
   Encoder drivetrainRightBackEncoder = null;

   //Defining Gyro
   Gyro drivetrainGyro = null;

   //Drivetrain Kinematics
   MecanumDriveOdometry drivetrainMecanumOdometry = null;

  public DriveTrain() {
    
    //Initalizing Moters
    leftFrontTalonFX = new WPI_TalonFX(DrivetrainConstants.DRIVETRAIN_LEFTFRONT_TALONFX);
    leftBackTalonFX = new WPI_TalonFX(DrivetrainConstants.DRIVETRAIN_LEFTBACK_TALONFX);
    rightBackTalonFX = new WPI_TalonFX(DrivetrainConstants.DRIVETRAIN_RIGHTBACK_TALONFX);
    rightFrontTalonFX = new WPI_TalonFX(DrivetrainConstants.DRIVETRAIN_RIGHTFRONT_TALONFX);

    //Creating deadband
    leftBackTalonFX.configNeutralDeadband(.06);
    rightBackTalonFX.configNeutralDeadband(.06);
    leftFrontTalonFX.configNeutralDeadband(.06);
    rightFrontTalonFX.configNeutralDeadband(.06);

    //Inverting Motors
    leftFrontTalonFX.setInverted(true);

    leftBackTalonFX.setInverted(true);

    //Initalizing mecanum drive
  mecanumDrive = new MecanumDrive(leftFrontTalonFX, leftBackTalonFX, rightFrontTalonFX, rightBackTalonFX);

    //Initalizing Drivetrain Encoders
    drivetrainLeftFrontEncoder = new Encoder(
      EncoderConstants.DRIVETRAIN_LEFTFRONT_ENCODER_01,
      EncoderConstants.DRIVETRAIN_LEFTFRONT_ENCODER_02,
      EncoderConstants.DRIVETRAIN_LEFTFRONT_ENCODER_ISREVERSED
    );

    drivetrainLeftBackEncoder = new Encoder(
      EncoderConstants.DRIVETRAIN_LEFTBACK_ENCODER_01,
      EncoderConstants.DRIVETRAIN_LEFTBACK_ENCODER_02,
      EncoderConstants.DRIVETRAIN_LEFTBACK_ENCODER_ISREVERSED
    );

    drivetrainRightFrontEncoder = new Encoder(
      EncoderConstants.DRIVETRAIN_RIGHTFRONT_ENCODER_01,
      EncoderConstants.DRIVETRAIN_RIGHTFRONT_ENCODER_02,
      EncoderConstants.DRIVETRAIN_RIGHTFRONT_ENCODER_ISREVERSED
    );

    drivetrainRightBackEncoder = new Encoder(
      EncoderConstants.DRIVETRAIN_RIGHTBACK_ENCODER_01,
      EncoderConstants.DRIVETRAIN_RIGHTBACK_ENCODER_02,
      EncoderConstants.DRIVETRAIN_RIGHTBACK_ENCODER_ISREVERSED
    );

    //Initalizing Drivetrain Gyro
    drivetrainGyro = new ADXRS450_Gyro();

    //Odometry class for tracking robot pose (Kinematics)
    drivetrainMecanumOdometry = new MecanumDriveOdometry(DrivetrainAutonomousConstants.DRIVETRAIN_MECANUM_KINEMATICS, drivetrainGyro.getRotation2d());

    // Set Encoder Distance per Pulse
    drivetrainLeftFrontEncoder.setDistancePerPulse(EncoderConstants.DRIVETRAIN_ENCODER_DISTANCE_PER_PULSE);
    drivetrainLeftBackEncoder.setDistancePerPulse(EncoderConstants.DRIVETRAIN_ENCODER_DISTANCE_PER_PULSE);
    drivetrainRightFrontEncoder.setDistancePerPulse(EncoderConstants.DRIVETRAIN_ENCODER_DISTANCE_PER_PULSE);
    drivetrainRightBackEncoder.setDistancePerPulse(EncoderConstants.DRIVETRAIN_ENCODER_DISTANCE_PER_PULSE);

  }
  
  public void cartesianDrive(double moveSpeedY, double moveSpeedX, double rotationSpeedZ, boolean gyroIsUsed) {
    

    if (gyroIsUsed) {
      mecanumDrive.driveCartesian(moveSpeedY, moveSpeedX, rotationSpeedZ, drivetrainGyro.getAngle());

    } 
    
    else {
      mecanumDrive.driveCartesian(moveSpeedY, moveSpeedX, rotationSpeedZ);

    }
  }

  public void setDriveMotorControllersVolts(MecanumDriveMotorVoltages inVolts) {

    // Drivetrain Action - Set the Voltage to each motor
    leftFrontTalonFX.setVoltage(inVolts.frontLeftVoltage);
    leftBackTalonFX.setVoltage(inVolts.rearLeftVoltage);
    rightFrontTalonFX.setVoltage(inVolts.frontRightVoltage);
    rightBackTalonFX.setVoltage(inVolts.rearRightVoltage);

  }

  public void resetEncoders() {

    // Reset each drivetrain motor encoder
    drivetrainLeftFrontEncoder.reset();
    drivetrainLeftBackEncoder.reset();
    drivetrainRightFrontEncoder.reset();
    drivetrainRightBackEncoder.reset();
  }

  public Pose2d getPose() {

    return drivetrainMecanumOdometry.getPoseMeters();
  }

   // Reset the position/orintation of the robot
  public void resetOdometry(Pose2d inPose) {

    drivetrainMecanumOdometry.resetPosition(inPose, drivetrainGyro.getRotation2d());
  }

  public Encoder getDrivetrainLeftFrontEncoder() {
    return drivetrainLeftFrontEncoder;
  }

  public Encoder getDrivetrainLeftBackEncoder() {
    return drivetrainLeftBackEncoder;
  }

  public Encoder getDrivetrainRightFrontEncoder() {
    return drivetrainRightFrontEncoder;
  }

  public Encoder getDrivetrainRightBackEncoder() {
    return drivetrainRightBackEncoder;
  }

  public MecanumDriveWheelSpeeds getCurrentDrivetrainWheelSpeeds() {
    return new MecanumDriveWheelSpeeds(
      drivetrainLeftFrontEncoder.getRate(), 
      drivetrainRightFrontEncoder.getRate(), 
      drivetrainLeftBackEncoder.getRate(), 
      drivetrainRightBackEncoder.getRate()
      );
  }

  public void setDrivetrainMaxOutput(double inMaxOutput) {

    mecanumDrive.setMaxOutput(inMaxOutput);
  }

  public void resetDrivetrainHeading() {

    drivetrainGyro.reset();
  }

  public double getDrivetrainHeading() {

    return drivetrainGyro.getRotation2d().getDegrees();
  }

  public double getDrivetrainTurnRate() {

    return -drivetrainGyro.getRate();
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    drivetrainMecanumOdometry.update(
      drivetrainGyro.getRotation2d(), 
      new MecanumDriveWheelSpeeds(
        drivetrainLeftFrontEncoder.getRate(), 
        drivetrainRightFrontEncoder.getRate(), 
        drivetrainLeftBackEncoder.getRate(), 
        drivetrainRightBackEncoder.getRate())
      );
    
  }
}
