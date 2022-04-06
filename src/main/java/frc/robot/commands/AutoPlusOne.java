// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDLights;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoPlusOne extends SequentialCommandGroup {
  /** Creates a new AutonPlusOne. */
  public AutoPlusOne(Shooter inSysShooter, Feeder inSysFeeder, LEDLights inSysLEDLights, DriveTrain inSysDriveTrain, Intake inSysIntake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelRaceGroup(
        new LowHood(inSysShooter),
        new AutoShootHigh(inSysShooter, inSysFeeder, inSysLEDLights)
      ),
     
      new CartesianDriveForAuton(inSysDriveTrain, 0, 0, .35, 1.15),
      
      new ParallelRaceGroup(
        new IntakeAndFeeder(inSysFeeder, inSysIntake),
        new CartesianDriveForAuton(inSysDriveTrain, -.25, 0, 0, 2.5)
        ),

      new CartesianDriveForAuton(inSysDriveTrain, 0, 0, .35, 1.15),

      new ParallelRaceGroup(
        new HighHood(inSysShooter),
        new AutoShootHigh(inSysShooter, inSysFeeder, inSysLEDLights)
      )
     
    );
  }
}