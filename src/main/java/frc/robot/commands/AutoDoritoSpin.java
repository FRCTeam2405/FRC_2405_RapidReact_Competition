// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.DoritoClimber;

public class AutoDoritoSpin extends CommandBase {

  private final DoritoClimber sysClimber;

  private static final int kCanID = ClimberConstants.MAIN_DORITO_SPARKMAX;
  private static final MotorType kMotorType = MotorType.kBrushless;
  private static final SparkMaxAlternateEncoder.Type kAltEncType = SparkMaxAlternateEncoder.Type.kQuadrature;
  private static final int kCPR = 8192;

  private CANSparkMax mainDoritoSparkMax;
  private SparkMaxPIDController m_pidController;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  /**
   * An alternate encoder object is constructed using the GetAlternateEncoder() 
   * method on an existing CANSparkMax object. If using a REV Through Bore 
   * Encoder, the type should be set to quadrature and the counts per 
   * revolution set to 8192
   */
  private RelativeEncoder mainDoritoEncoder;

  /** Creates a new ClimberStage1. */
  public AutoDoritoSpin(DoritoClimber inSysClimber)  {
    sysClimber = inSysClimber;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(sysClimber);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // initialize SPARK MAX with CAN ID
    mainDoritoSparkMax = new CANSparkMax(kCanID, kMotorType);
    mainDoritoSparkMax.restoreFactoryDefaults();

    mainDoritoEncoder = mainDoritoSparkMax.getAlternateEncoder(kAltEncType, kCPR);
    
    /**
     * In order to use PID functionality for a controller, a SparkMaxPIDController object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
    m_pidController = mainDoritoSparkMax.getPIDController();
  
    /**
     * By default, the PID controller will use the Hall sensor from a NEO for its
     * feedback device. Instead, we can set the feedback device to the alternate
     * encoder object
     */
    m_pidController.setFeedbackDevice(mainDoritoEncoder);

    /**
     * From here on out, code looks exactly like running PID control with the 
     * built-in NEO encoder, but feedback will come from the alternate encoder
     */ 

    // PID coefficients
    kP = 0.1; 
    kI = 1e-4;
    kD = 1; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 1; 
    kMinOutput = -1;

    // set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Set Rotations", 0);

   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    double rotations = SmartDashboard.getNumber("Set Rotations", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { m_pidController.setP(p); kP = p; }
    if((i != kI)) { m_pidController.setI(i); kI = i; }
    if((d != kD)) { m_pidController.setD(d); kD = d; }
    if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_pidController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; }

    m_pidController.setReference(rotations, CANSparkMax.ControlType.kPosition);
    
    SmartDashboard.putNumber("SetPoint", rotations);
    SmartDashboard.putNumber("ProcessVariable", mainDoritoEncoder.getPosition());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  
}
