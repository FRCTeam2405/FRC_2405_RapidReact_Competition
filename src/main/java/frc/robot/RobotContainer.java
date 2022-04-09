// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ControllerConstants.SecondaryDriver;
import frc.robot.commands.ClimberStage1;
import frc.robot.commands.ClimberStage2;
import frc.robot.commands.ClimberStage3;
import frc.robot.commands.ClimberStageOneRelease;
import frc.robot.commands.ClimberStageThreeRelease;
import frc.robot.commands.ClimberStageTwoRelease;
import frc.robot.commands.DoritoLifter;
import frc.robot.commands.DoritoLower;
import frc.robot.commands.DoritoMotor;
import frc.robot.commands.AutoBasicOne;
import frc.robot.commands.AutoCargoOne;
import frc.robot.commands.AutoDoritoClimb;
import frc.robot.commands.AutoDoritoSpin;
import frc.robot.commands.AutoPlusOne;
import frc.robot.commands.AutoTestOne;
import frc.robot.commands.ClimberDisablingShooter;
import frc.robot.commands.DriveAuto;
import frc.robot.commands.DriveMecanum;
import frc.robot.commands.FeedCargo;
import frc.robot.commands.FeedShooter;
import frc.robot.commands.HighHood;
import frc.robot.commands.IntakeAndFeeder;
import frc.robot.commands.IntakeAndFeederReversed;
import frc.robot.commands.IntakeDeploy;
import frc.robot.commands.IntakeDeployReversed;
import frc.robot.commands.IntakeRetract;
import frc.robot.commands.JoystickDoritoSpinner;
import frc.robot.commands.LEDDeclare;
import frc.robot.commands.LiftClimber;
import frc.robot.commands.LowHood;
import frc.robot.commands.ShootCargoHigh;
import frc.robot.commands.ShootCargoLow;
import frc.robot.commands.ShootHigh;
import frc.robot.commands.ShootLow;
import frc.robot.commands.ShooterOff;
import frc.robot.commands.TimeWait;
import frc.robot.subsystems.AutoTrajectory;
import frc.robot.subsystems.CameraSystem;
import frc.robot.subsystems.DoritoClimber;
import frc.robot.subsystems.DoritoSpin;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDLights;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  
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
  public static final Feeder m_Feeder = new Feeder();

  //Adding LED's to RobotContainer
  public static final LEDLights m_LEDsetting = new LEDLights();

  //Climber subsystem
  public static final DoritoClimber m_doritoclimber = new  DoritoClimber();
  //Dorito Motor subsystem
  public static final DoritoSpin m_DoritoSpin = new DoritoSpin();

  //Camera subsystem
  public static final CameraSystem m_camerasystem = new CameraSystem();

  public static final AutoTrajectory m_autotrajectory = new AutoTrajectory();

//-----------------------------------------------------------------------------

  //----------
  // Commands
  //----------

  // Timer Commands (Wait)
  private final TimeWait cmdTimeWait3Sec = new TimeWait(3);
  private final TimeWait cmdTimeWait5Sec = new TimeWait(5);

  //Defining Intake Commands
  private final IntakeDeploy cmdIntakeDeploy = new IntakeDeploy(m_Intake);
  private final IntakeRetract cmdIntakeRetract = new IntakeRetract(m_Intake);

  private final IntakeAndFeeder cmdIntakeAndFeeder = new IntakeAndFeeder(m_Feeder, m_Intake);
  private final IntakeAndFeederReversed cmdIntakeAndFeederReversed = new IntakeAndFeederReversed(m_Feeder, m_Intake);
  
  //Defining Feeder Command
  private final FeedCargo cmdFeedCargo = new FeedCargo(m_Feeder);
  private final FeedShooter cmdFeedShooter = new FeedShooter(m_Feeder); 

  //Autonomous Commands
  private final DriveAuto cmdDriveAuto = new DriveAuto(m_DriveTrain);

  //AutoTestOne Command
  private final AutoTestOne cmdAutoTestOne = new AutoTestOne(m_Shooter, m_Feeder, m_DriveTrain, m_LEDsetting);

  private final IntakeDeployReversed cmdIntakeDeployReversed = new IntakeDeployReversed(m_Intake);

  //Hood Commands
  private final HighHood cmdHighHood = new HighHood(m_Shooter);
  private final LowHood cmdLowHood = new LowHood(m_Shooter);

  //-------------------
  // LED Commands
  //-------------------

  //LED Commands
  private final LEDDeclare cmdLEDLightsIntake = new LEDDeclare(m_LEDsetting, -.87);
  private final LEDDeclare cmdLEDLightsShootLow = new LEDDeclare(m_LEDsetting, -.87);
  private final LEDDeclare cmdLEDLightsShootHigh = new LEDDeclare(m_LEDsetting, -.87);
  private final LEDDeclare cmdLEDLightsLEDs = new LEDDeclare(m_LEDsetting, SmartDashboard.getNumber("LEDSet", -.69));

  //------------------------------------------------------------------------

  //------------------
  // Shooter Commands
  //------------------

  //Adding "ShootCargoHigh"
  private final ShootCargoHigh cmdShootCargoHigh = new ShootCargoHigh(m_Shooter, m_Feeder, m_LEDsetting);

  //Adding "ShootCargoLow"
  private final ShootCargoLow cmdShootCargoLow = new ShootCargoLow(m_Shooter, m_Feeder, m_LEDsetting);

  //Defining Shoot Commands
  private final ShootHigh cmdShootHigh = new ShootHigh(m_Shooter, m_LEDsetting);
  private final ShootLow cmdShootLow = new ShootLow(m_Shooter, m_LEDsetting);

  //ShooterOff
  private final ShooterOff cmdShooterOff = new ShooterOff(m_Shooter, m_LEDsetting);

  //------------------------------------------------------------------------

  //-----------------
  // Dorito Commands
  //-----------------

  //Dorito Clamp commands
  private final ClimberStage1 clampOne = new ClimberStage1(m_doritoclimber, m_LEDsetting);
  private final ClimberStage2 clampTwo = new ClimberStage2(m_doritoclimber, m_LEDsetting);
  private final ClimberStage3 clampThree = new ClimberStage3(m_doritoclimber, m_LEDsetting);
  //Dorito Unclamp Commands
  private final ClimberStageOneRelease cmdReleaseClampOne = new ClimberStageOneRelease(m_doritoclimber, m_LEDsetting);
  private final ClimberStageTwoRelease cmdReleaseClampTwo = new ClimberStageTwoRelease(m_doritoclimber, m_LEDsetting);
  private final ClimberStageThreeRelease cmdReleaseClampThree = new ClimberStageThreeRelease(m_doritoclimber, m_LEDsetting);
  //Dorito Lifter Command
  private final DoritoLifter cmdDoritoLifterEngage = new DoritoLifter(m_doritoclimber, m_LEDsetting);
  private final LiftClimber cmdLiftClimber = new LiftClimber(m_doritoclimber, m_LEDsetting, m_DoritoSpin);

  private final ClimberDisablingShooter cmdClimberDisablingShooter = new ClimberDisablingShooter(m_doritoclimber, m_LEDsetting, m_Shooter, m_DoritoSpin);
  //Dorito Lower Command
  private final DoritoLower cmdDoritoLowerEngage = new DoritoLower(m_doritoclimber, m_LEDsetting);
  //Dorito Motor Command
  private final DoritoMotor cmdDoritoMotorEngage = new DoritoMotor(m_DoritoSpin);

  private final JoystickDoritoSpinner cmdJoystickDoritoSpinner = new JoystickDoritoSpinner(m_DoritoSpin, secondarycontroller);
  //AutoDoritoSpin Command
  private final AutoDoritoSpin cmdAutoDoritoSpin = new AutoDoritoSpin(m_doritoclimber);
  //AutoDoritoClimb
  private final AutoDoritoClimb cmdAutoDoritoClimb = new AutoDoritoClimb(m_doritoclimber, m_LEDsetting);
//Auto Commands
  private final AutoBasicOne cmdAutoBasicOne = new AutoBasicOne(m_Shooter, m_Feeder, m_LEDsetting, m_DriveTrain, m_autotrajectory);
  private final AutoCargoOne cmdAutoCargoOne = new AutoCargoOne(m_Shooter, m_Feeder, m_LEDsetting, m_DriveTrain, m_autotrajectory);
//Auton command for spinning
  private final AutoPlusOne cmdAutoPlusOne = new AutoPlusOne(m_Shooter, m_Feeder, m_LEDsetting, m_DriveTrain, m_Intake);

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
  public static final Joystick secondarycontroller = new Joystick(SecondaryDriver.SECONDARYDRIVER_CONTROLLER);
  public static final GenericHID driverSecondaryYAxis = new GenericHID(SecondaryDriver.SECONDARYDRIVER_CONTROLLER_MOVE_AXISY);
  public static final JoystickButton driversecondarybuttonBlue = new JoystickButton(secondarycontroller, ControllerConstants.SecondaryDriver.SECONDARYDRIVER_CONTROLLER_BUTTON_BLUE);
  public static final JoystickButton driversecondarybuttonGreen= new JoystickButton(secondarycontroller, ControllerConstants.SecondaryDriver.SECONDARYDRIVER_CONTROLLER_BUTTON_GREEN);
  public static final JoystickButton driversecondarybuttonRed = new JoystickButton(secondarycontroller, ControllerConstants.SecondaryDriver.SECONDARYDRIVER_CONTROLLER_BUTTON_RED);
  public static final JoystickButton driverSecondaryButtonYellow = new JoystickButton(secondarycontroller, ControllerConstants.SecondaryDriver.SECONDARYDRIVER_CONTROLLER_BUTTON_YELLOW);
  public static final JoystickButton driverSecondarySwitchZero = new JoystickButton(secondarycontroller, ControllerConstants.SecondaryDriver.SECONDARYDRIVER_CONTROLLER_SWITCH_0);
  public static final JoystickButton driverSecondarySwitchOne = new JoystickButton(secondarycontroller, SecondaryDriver.SECONDARYDRIVER_CONTROLLER_SWITCH_1);
  public static final JoystickButton driverSecondarySwitchTwo = new JoystickButton(secondarycontroller, SecondaryDriver.SECONDARYDRIVER_CONTROLLER_SWITCH_2);
  public static final JoystickButton driverSecondarySwitchThree = new JoystickButton(secondarycontroller, SecondaryDriver.SECONDARYDRIVER_CONTROLLER_SWITCH_3);

  //------------------------------------------------------------------------------------------

  //Defining PCM
  private final Compressor PCMCompressor = new Compressor(IntakeConstants.PORT_PCM_MAIN, PneumaticsModuleType.CTREPCM);

  private SendableChooser<Command> dropdownCommandChooserAuton = new SendableChooser<>();
  private SendableChooser<Command> dropdownCommandChooserTeleop = new SendableChooser<>();
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
  // Configure the button bindings
    configureButtonBindings();
   
   //Declaring commands in Robot Container
   m_DriveTrain.setDefaultCommand(new DriveMecanum(m_DriveTrain));
   m_DoritoSpin.setDefaultCommand(new JoystickDoritoSpinner(m_DoritoSpin, secondarycontroller));

   //Testing feeder motor speed on the dashbord 
   SmartDashboard.putNumber("IntakePercentOutput", IntakeConstants.INTAKE_DEFAULT_SPEED);
   SmartDashboard.getNumber("IntakePercentOutputReverced", IntakeConstants.INTAKE_DEFAULT_SPEED_REVERSED);
   SmartDashboard.putNumber("FeederPercentOutput", FeederConstants.FEEDER_DEFAULT_OUTPUT);
   SmartDashboard.putNumber("ShootHighPercentOutput", ShooterConstants.SHOOTHIGH_DEFAULT_OUTPUT);
   SmartDashboard.putNumber("ShootLowPercentOutput", ShooterConstants.SHOOTLOW_DEFAULT_OUTPUT);
   SmartDashboard.putNumber("LEDSet", LEDConstants.LED_SETTING_DEFAULT);
   SmartDashboard.putNumber("doritoSpinner", ClimberConstants.DORITO_DEFAULT_SPEED);
   SmartDashboard.putNumber("ShooterHood", ShooterConstants.SHOOTER_HOOD_DEFAULT_OUTPUT);
   SmartDashboard.putBoolean("LimitSwitchTripped", m_Feeder.LimitSwitchTripped());
   SmartDashboard.putNumber("Auton Time", DrivetrainConstants.TIME_DRIVING_IN_AUTON);
   SmartDashboard.putNumber("AutonSpeed", DrivetrainConstants.VELOCITY_DRIVING_IN_AUTON);
   SmartDashboard.putString("Shooter State", "_");
   SmartDashboard.putString("Hood State", "_");

   dropdownCommandChooserAuton.setDefaultOption("Basic One", cmdAutoBasicOne);
   dropdownCommandChooserAuton.addOption("Auto 2 Ball", cmdAutoPlusOne);
  // dropdownCommandChooser.addOption("Shoot High", cmdShootCargoHigh);
   //dropdownCommandChooserAuton.setDefaultOption("Basic One", cmdAutoBasicOne);
   //dropdownCommandChooserAuton.addOption("Shoot Low", cmdShootCargoLow);
   //dropdownCommandChooser.addOption("Shoot High", cmdShootCargoHigh);
   //dropdownCommandChooser.addOption("Auto Cargo One", cmdAutoCargoOne);
  
   SmartDashboard.putData(dropdownCommandChooserAuton);


   dropdownCommandChooserTeleop.setDefaultOption("Shoot No", cmdShooterOff);
   dropdownCommandChooserTeleop.addOption("Shoot High", cmdShootLow);
   dropdownCommandChooserTeleop.addOption("Shoot High", cmdShootHigh);

   SmartDashboard.putData(dropdownCommandChooserTeleop);
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
   //driverMainButtonX.whenHeld(cmdIntakeAndFeeder, true);//.andThen(cmdLEDLightsIntake));
   //driverMainButtonB.whenHeld(cmdIntakeAndFeederReversed, true);
   driverMainButtonY.whenHeld(cmdFeedShooter, true);//.andThen(cmdLEDLightsShootHigh));
   //driverMainButtonA.whenHeld(new LEDDeclare(m_LEDsetting, SmartDashboard.getNumber("LEDSet", -.69)), true); //.andThen(cmdFeedCargo).andThen(cmdLEDLightsShootLow), true);
   driverMainBumperRight.whenHeld(cmdIntakeAndFeeder, true);
   driverMainBumperLeft.whenHeld(cmdIntakeAndFeederReversed, true);
  
   //Secondary Button Mappings
   driversecondarybuttonBlue.whenPressed(cmdShootLow);
   driversecondarybuttonGreen.whenPressed(cmdShootHigh);
   driversecondarybuttonRed.whenHeld(cmdHighHood);
   driverSecondaryButtonYellow.whenHeld(cmdLowHood); 
   driverSecondarySwitchZero.whenActive(cmdClimberDisablingShooter);
   driverSecondarySwitchZero.whenInactive(cmdDoritoLowerEngage);
   driverSecondarySwitchOne.whenActive(cmdReleaseClampOne);
   driverSecondarySwitchOne.whenInactive(clampOne);
   driverSecondarySwitchTwo.whenActive(cmdReleaseClampTwo);
   driverSecondarySwitchTwo.whenInactive(clampTwo);
   driverSecondarySwitchThree.whenActive(cmdReleaseClampThree);
   driverSecondarySwitchThree.whenInactive(clampThree);
   //secondarycontroller.(cmdJoystickDoritoSpinner); <-- Makes the spinner spin
   
  // if (driverSecondaryYAxis.getRawAxis(1) != 0) {
  // new JoystickDoritoSpinner(m_doritoclimber, secondarycontroller);
  // }
  
  }

  //---------------------------------------------------------------------

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
   // return cmdShootCargoHigh;
    return dropdownCommandChooserAuton.getSelected();
  //return null;
  }
  public Command getTeleopCommand() {
     return dropdownCommandChooserTeleop.getSelected();
  }

}