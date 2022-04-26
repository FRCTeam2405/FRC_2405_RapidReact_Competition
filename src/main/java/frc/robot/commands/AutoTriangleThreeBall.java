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
public class AutoTriangleThreeBall extends SequentialCommandGroup {
  /** Creates a new AutoTriangleThreeBall. */
  public AutoTriangleThreeBall(Shooter inSysShooter, Feeder inSysFeeder, LEDLights inSysLEDLights, DriveTrain inSysDriveTrain, Intake inSysIntake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    //Shoots Ball
    new AutoShootHigh(inSysShooter, inSysFeeder, inSysLEDLights),

    //Turns a 180
    new CartesianDriveForAuton(inSysDriveTrain, 0, 0, .35, 1.04),

    //Drives until the time on the drive command runs out
    new ParallelRaceGroup(
      new IntakeAndFeeder(inSysFeeder, inSysIntake),
    //Drive 9.4 feet
      new CartesianDriveForAuton(inSysDriveTrain,  -.25, 0, 0, 2.5)
      ),
    //Back up 2.1 feet 
    new CartesianDriveForAuton(inSysDriveTrain, .134, 0, 0, 2),

    //Turn 90 degrees
    new CartesianDriveForAuton(inSysDriveTrain, 0, 0, .35, .53),

    //Drive forward to next ball and pick it up
    new ParallelRaceGroup(
      new IntakeAndFeeder(inSysFeeder, inSysIntake),
      new CartesianDriveForAuton(inSysDriveTrain, -.25, 0, 0, 2.12)
    ),
   
    //Rotate a little bit over 90 degrees
    new CartesianDriveForAuton(inSysDriveTrain, 0, 0, .35, .65),

    //Drive into hub
    new CartesianDriveForAuton(inSysDriveTrain, -.33, 0, 0, 1.20),

    //Shoots ball
    new AutoShootHigh(inSysShooter, inSysFeeder, inSysLEDLights)

    );
  }
}
