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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DrivetrainAutonomousConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.EncoderConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.ClimberStage1;
import frc.robot.commands.ClimberStage2;
import frc.robot.commands.ClimberStage3;
import frc.robot.commands.DoritoLifter;
import frc.robot.commands.DoritoMotor;
import frc.robot.commands.DriveAuto;
import frc.robot.commands.DriveMecanum;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.FeedCargo;
import frc.robot.commands.IntakeDeploy;
import frc.robot.commands.IntakeRetract;
import frc.robot.commands.LEDDeclare;
import frc.robot.commands.ShootHigh;
import frc.robot.commands.ShootLow;
import frc.robot.subsystems.DoritoClimber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDLights;
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

  //------------
  // Subsystems
  //------------

  //Adding DriveTrain subsystem to RobotContainer
  public static final DriveTrain m_DriveTrain = new DriveTrain();

  //Adding Intake to RobotContainer
  public static final Intake m_Intake = new Intake();

  //Adding Shooter to RobotContainer
  public static final Shooter m_Shooter = new Shooter();

  //Adding Feeder to RobotContainer
  public static final Feeder m_feedermotor = new Feeder();

  //Adding LED's to RobotContainer
  public static final LEDLights m_LEDsetting = new LEDLights();

  //Climber subsystem
  public static final DoritoClimber m_doritoclimber = new  DoritoClimber();

//-----------------------------------------------------------------------------

  //----------
  // Commands
  //----------

  //Defining Intake Commands
  private final IntakeDeploy cmdIntakeDeploy = new IntakeDeploy(m_Intake);
  private final IntakeRetract cmdIntakeRetract = new IntakeRetract(m_Intake);
  
  //Defining Shoot Commands
  private final ShootHigh cmdShootHigh = new ShootHigh(m_Shooter);
  private final ShootLow cmdShootLow = new ShootLow(m_Shooter);

  //Defining Feeder Command
  private final FeedCargo cmdFeedCargo = new FeedCargo(m_feedermotor);

  //Autonomous Commands
  private final DriveAuto cmdDriveAuto = new DriveAuto();

  //LED Commands
  private final LEDDeclare cmdLEDLightsIntake = new LEDDeclare(m_LEDsetting, -.87);
  private final LEDDeclare cmdLEDLightsShootLow = new LEDDeclare(m_LEDsetting, -.87);
  private final LEDDeclare cmdLEDLightsShootHigh = new LEDDeclare(m_LEDsetting, -.87);
  private final LEDDeclare cmdLEDLightsLEDs = new LEDDeclare(m_LEDsetting, SmartDashboard.getNumber("LEDSet", -.69));

  //Dorito Clamp commands
  private final ClimberStage1 clampOne = new ClimberStage1(m_doritoclimber);
  private final ClimberStage2 clampTwo = new ClimberStage2(m_doritoclimber);
  private final ClimberStage3 clampThree = new ClimberStage3(m_doritoclimber);
  //Dorito Lifter Command
  private final DoritoLifter cmdDoritoLifterEngage = new DoritoLifter(m_doritoclimber);
  //Dorito Motor Command
  private final DoritoMotor cmdDoritoMotorEngage = new DoritoMotor(m_doritoclimber);

  //---------------------------------------------------------------------------------------

  //-----------------
  // Xboxcontroller
  
  //-----------------

  //Defining Xboxcontroller
  public static final XboxController driverController = new XboxController(ControllerConstants.Driver.DRIVER_CONTROLLER);
  private final JoystickButton driverMainButtonX = new JoystickButton(driverController, ControllerConstants.Driver.DRIVER_CONTROLLER_BUTTON_X);
  private final JoystickButton driverMainButtonB = new JoystickButton(driverController, ControllerConstants.Driver.DRIVER_CONTROLLER_BUTTON_B);
  private final JoystickButton driverMainButtonY = new JoystickButton(driverController, ControllerConstants.Driver.DRIVER_CONTROLLER_BUTTON_Y);
  private final JoystickButton driverMainButtonA = new JoystickButton(driverController, ControllerConstants.Driver.DRIVER_CONTROLLER_BUTTON_A);
  private final JoystickButton driverMainBumperRight = new JoystickButton(driverController, ControllerConstants.Driver.DRIVER_CONTROLLER_BUMPER_RIGHT);
  private final JoystickButton driverMainBumperLeft = new JoystickButton(driverController, ControllerConstants.Driver.DRIVER_CONTROLLER_BUMPER_LEFT);

  //Defining Arcade Controller
  public static final Joystick secondarycontroller = new Joystick(1);
  public static final JoystickButton driversecondarybuttonX = new JoystickButton(secondarycontroller, ControllerConstants.SecondaryDriver.SECONDARYDRIVER_CONTROLLER_BUTTON_X);
  public static final JoystickButton driversecondarybuttonA = new JoystickButton(secondarycontroller, ControllerConstants.SecondaryDriver.SECONDARYDRIVER_CONTROLLER_BUTTON_A);
  public static final JoystickButton driversecondarybuttonB = new JoystickButton(secondarycontroller, ControllerConstants.SecondaryDriver.SECONDARYDRIVER_CONTROLLER_BUTTON_B);
  public static final JoystickButton driverSecondaryButtonY = new JoystickButton(secondarycontroller, ControllerConstants.SecondaryDriver.SECONDARYDRIVER_CONTROLLER_BUTTON_Y);
  public static final JoystickButton driverSecondaryLeftBumper = new JoystickButton(secondarycontroller, ControllerConstants.SecondaryDriver.SECONDARYDRIVER_CONTROLLER_LEFT_BUMPER);

  //------------------------------------------------------------------------------------------


  //Defining PCM
  private final Compressor PCMCompressor = new Compressor(IntakeConstants.PORT_PCM_MAIN, PneumaticsModuleType.REVPH);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
   
   //Declaring commands in Robot Container
   m_DriveTrain.setDefaultCommand(new DriveMecanum(m_DriveTrain));

   //Testing feeder otor speed on the dashbord 
   SmartDashboard.putNumber("IntakePercentOutput", -0.75);
   SmartDashboard.putNumber("FeederPercentOutput", -0.50);
   SmartDashboard.putNumber("ShooterPercentOutput", 0.35);
   SmartDashboard.putNumber("LEDSet", -.69);
   SmartDashboard.putNumber("doritoSpinner", -.69);

  }
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

  //----------------------------
  //Configuring Button Bindings
  //----------------------------

  private void configureButtonBindings() {
    
   //Button Mappings
   driverMainButtonX.whenPressed(cmdIntakeDeploy.andThen(cmdLEDLightsIntake));
   driverMainButtonB.whenPressed(cmdIntakeRetract);
   driverMainButtonY.whenPressed(cmdShootHigh.andThen(cmdFeedCargo).andThen(cmdLEDLightsShootHigh));
   driverMainButtonA.toggleWhenPressed(cmdShootLow.andThen(cmdFeedCargo).andThen(cmdLEDLightsShootLow));
   driverMainBumperRight.toggleWhenPressed(cmdFeedCargo, true);
   driverMainBumperLeft.toggleWhenPressed(cmdLEDLightsLEDs, true);
  
   //Secondary Button Mappings
   driversecondarybuttonX.toggleWhenPressed(clampOne);
   driversecondarybuttonA.toggleWhenPressed(clampTwo);
   driversecondarybuttonB.toggleWhenPressed(clampThree);
   driverSecondaryButtonY.toggleWhenPressed(cmdDoritoLifterEngage);
   driverSecondaryLeftBumper.toggleWhenPressed(cmdDoritoMotorEngage);
  }
  //----------------------------------------------------------------------

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