// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.LEDLights;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoTestOne extends SequentialCommandGroup {
  /** Creates a new AutoTestOne. */
  public AutoTestOne(Shooter m_Shooter, Feeder m_Feeder, DriveTrain m_Drivetrain, LEDLights inSysLedLights) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ShootLow(m_Shooter, inSysLedLights),

      new TimeWait(3),

      new FeedCargo(m_Feeder)//,
      
    //  new DriveAuto(m_Drivetrain)


    );
  }
}
