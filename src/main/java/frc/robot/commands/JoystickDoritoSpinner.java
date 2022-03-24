// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ControllerConstants.SecondaryDriver;
import frc.robot.subsystems.DoritoClimber;
import frc.robot.subsystems.DoritoSpin;

public class JoystickDoritoSpinner extends CommandBase {
  /** Creates a new JoystickDoritoSpinner. */
  private final DoritoSpin sysClimber;
  private final GenericHID sysJoystick;

  public JoystickDoritoSpinner(DoritoSpin inSysClimber, Joystick inJoystick) {
    sysClimber = inSysClimber;
    sysJoystick = inJoystick;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(sysClimber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double doritoRotateSpeed = RobotContainer.secondarycontroller.getY()* (ClimberConstants.DORITO_DEFAULT_SPEED);

    //sysClimber.DoritoClimberMotor(sysJoystick.getRawAxis(SecondaryDriver.SECONDARYDRIVER_CONTROLLER_MOVE_AXISY));
    sysClimber.DoritoClimberMotor(doritoRotateSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   // if (SmartDashboard.putNumber("doritoSpinner", ClimberConstants.DORITO_DEFAULT_SPEED))
   //   return true;
    //else
      return false;
  }
}
