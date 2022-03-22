// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DoritoClimber;
import frc.robot.subsystems.LEDLights;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoDoritoClimb extends SequentialCommandGroup {
  /** Creates a new DoritoAutoClimb. */
  public AutoDoritoClimb(DoritoClimber m_doritoclimber, LEDLights m_LEDLights) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      
//Activate clamp 1
new ClimberStage1(m_doritoclimber, m_LEDLights),
//Spin to bar 3
new AutoDoritoSpin(m_doritoclimber),
//Activate clamp 2
new ClimberStage2(m_doritoclimber, m_LEDLights),
//Release clamp 1
new ClimberStageOneRelease(m_doritoclimber, m_LEDLights),
//Spin to bar 4
new AutoDoritoSpin(m_doritoclimber),
//Activate clamp 3
new ClimberStage3(m_doritoclimber, m_LEDLights),
//Release clamp 2
new ClimberStageTwoRelease(m_doritoclimber, m_LEDLights)


    );
  }
}
