// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DrivetrainAutonomousConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.EncoderConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.DriveAuto;
import frc.robot.commands.DriveMecanum;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.FeedCargo;
import frc.robot.commands.IntakeDeploy;
import frc.robot.commands.IntakeRetract;
import frc.robot.commands.ShootHigh;
import frc.robot.commands.ShootLow;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  //Adding DriveTrain subsystem to RobotContainer
  public static final DriveTrain m_DriveTrain = new DriveTrain();

  //Adding Intake to RobotContainer
  public static final Intake m_Intake = new Intake();

  //Adding Shooter to RobotContainer
  public static final Shooter m_Shooter = new Shooter();

  //Adding Feeder to RobotContainer
  public static final Feeder m_feedermotor = new Feeder();

  //Defining Intake Commands
  private final IntakeDeploy cmdIntakeDeploy = new IntakeDeploy();
  private final IntakeRetract cmdIntakeRetract = new IntakeRetract();
  
  //Defining Shoot Commands
  private final ShootHigh cmdShootHigh = new ShootHigh();
  private final ShootLow cmdShootLow = new ShootLow();

  //Defining Feeder Command
  private final FeedCargo cmdFeedCargo = new FeedCargo();

  //Autonomous Commands
  private final DriveAuto cmdDriveAuto = new DriveAuto();

  //Defining Xboxcontroller
  public static final XboxController driverController = new XboxController(ControllerConstants.DRIVER_CONTROLLER);
  private final JoystickButton driverMainButtonX = new JoystickButton(driverController, ControllerConstants.DRIVER_CONTROLLER_BUTTON_X);
  private final JoystickButton driverMainButtonB = new JoystickButton(driverController, ControllerConstants.DRIVER_CONTROLLER_BUTTON_B);
  private final JoystickButton driverMainButtonY = new JoystickButton(driverController, ControllerConstants.DRIVER_CONTROLLER_BUTTON_Y);
  private final JoystickButton driverMainButtonA = new JoystickButton(driverController, ControllerConstants.DRIVER_CONTROLLER_BUTTON_A);
  private final JoystickButton driverMainBumperRight = new JoystickButton(driverController, ControllerConstants.DRIVER_CONTROLLER_BUMPER_RIGHT);

  //Defining PCM
  private final Compressor PCMCompressor = new Compressor(IntakeConstants.PORT_PCM_MAIN, PneumaticsModuleType.REVPH);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
   
   //Declaring commands in Robot Container
   m_DriveTrain.setDefaultCommand(new DriveMecanum());

  }
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    
   //Button Mappings
   driverMainButtonX.whenPressed(cmdIntakeDeploy);
   driverMainButtonB.whenPressed(cmdIntakeRetract);
   driverMainButtonY.whenPressed(cmdShootHigh);
   driverMainButtonA.whenPressed(cmdShootLow);
   driverMainBumperRight.toggleWhenPressed(cmdFeedCargo, true);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
//    return cmdDriveAuto;
    return null;
  }
  

}