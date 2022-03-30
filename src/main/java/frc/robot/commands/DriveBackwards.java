// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.LEDLights;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveBackwards extends ParallelDeadlineGroup {
        //private static double basicOneTime = SmartDashboard.getNumber("Auton Time", DrivetrainConstants.TIME_DRIVING_IN_AUTON);
  /** Creates a new DrivBackwards. */
  public DriveBackwards(DriveTrain sysDrivetrain) {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(new TimeWait(2));
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new CartesianDriveForAuton(sysDrivetrain)
      );
  }
}
