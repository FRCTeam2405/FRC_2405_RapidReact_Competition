// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    //CAN ports
    private static final int CAN_NETWORK_PORT_10 = 10;
    private static final int CAN_NETWORK_PORT_11 = 11;
    private static final int CAN_NETWORK_PORT_12 = 12;
    private static final int CAN_NETWORK_PORT_13 = 13;

public static final class DrivetrainConstants{
    //Assining CAN ports to Talons
    public static final int DRIVETRAIN_LEFTFRONT_TALONFX = CAN_NETWORK_PORT_10;
    public static final int DRIVETRAIN_RIGHTFRONT_TALONFX = CAN_NETWORK_PORT_11;
    public static final int DRIVETRAIN_LEFTBACK_TALONFX = CAN_NETWORK_PORT_12;
    public static final int DRIVETRAIN_RIGHTBACK_TALONFX = CAN_NETWORK_PORT_13;

    

}

public static final class DrivetrainAutonomousConstants {
    //Max Speed per Second in Meters
    public static final double DRIVETRAIN_MAX_SPEED_PER_SECOND_METERS = 3; //Need to Change
    
    //Max Acceleration per Second
    public static final double DRIVETRAIN_MAX_ACCELERATION_PER_SECOND_SQUARED_METERS = 3; //Need to Change
    
    public static final double DRIVETRAIN_CONTROLLER_PY = .5;  //Need to Change
    public static final double DRIVETRAIN_CONTROLLER_PX = .5;  //Need to Change
    public static final double DRIVETRAIN_CONTROLLER_PTHETA = .5;  //Need to Change

    private static final double DRIVETRAIN_MAX_ANGULAR_SPEED_PER_SECOND_RADIANS = 0;

    private static final double DRIVETRAIN_MAX_ANGULAR_SPEED_PER_SECOND_SQUARED_RADIANS = 0;
   
    public static final TrapezoidProfile.Constraints DRIVETRAIN_THETA_CONTROLLER_CONSTRAINTS = 
    new TrapezoidProfile.Constraints(
        DRIVETRAIN_MAX_ANGULAR_SPEED_PER_SECOND_RADIANS, 
        DRIVETRAIN_MAX_ANGULAR_SPEED_PER_SECOND_SQUARED_RADIANS
        );

    private static final double DRIVETRAIN_WHEEL_BASE_METERS = 0.5715;
    private static final double DRIVETRAIN_TRACK_WIDTH_METERS = 0.5461;
    
    //MecanumDrive Kinematics
    public static final MecanumDriveKinematics DRIVETRAIN_MECANUM_KINEMATICS = 
new MecanumDriveKinematics(
    new Translation2d(DRIVETRAIN_WHEEL_BASE_METERS / 2, DRIVETRAIN_TRACK_WIDTH_METERS / 2), // Front Left (meters)
    new Translation2d(DRIVETRAIN_WHEEL_BASE_METERS / 2, -DRIVETRAIN_TRACK_WIDTH_METERS / 2), // Front Right (meters)
    new Translation2d(-DRIVETRAIN_WHEEL_BASE_METERS / 2, DRIVETRAIN_TRACK_WIDTH_METERS / 2), // Back Left (meters)
    new Translation2d(-DRIVETRAIN_WHEEL_BASE_METERS / 2, -DRIVETRAIN_TRACK_WIDTH_METERS / 2));  // Back Right (meters);)  
    

    
    //Wheel Diameter
    public static final double DRIVETRAIN_WHEEL_DIAMETER_METERS = 0.1524;
    
    //Track width 21.5
    //Wheel Base 22.5
}

public static final class EncoderConstants {
    //Encoder is Reversed
    public static final boolean DRIVETRAIN_LEFTFRONT_ENCODER_ISREVERSED = true;
    public static final boolean DRIVETRAIN_LEFTBACK_ENCODER_ISREVERSED = true;
    public static final boolean DRIVETRAIN_RIGHTFRONT_ENCODER_ISREVERSED = false;
    public static final boolean DRIVETRAIN_RIGHTBACK_ENCODER_ISREVERSED = false;

    //Encoder distance per pulse
    public static final double DRIVETRAIN_ENCODER_DISTANCE_PER_PULSE = 
    (DrivetrainAutonomousConstants.DRIVETRAIN_WHEEL_DIAMETER_METERS * Math.PI) / (double) EncoderConstants.DRIVETRAIN_ENCODER_CYCLES_PER_ROTATION;

    //Encoder Cycles per Rotation
    public static final int DRIVETRAIN_ENCODER_CYCLES_PER_ROTATION = 2048;

    //Encoder Velocity Percent
    public static final double DRIVETRAIN_ENCODER_FRONT_LEFT_VELOCITY_PERCENT = .5;
    public static final double DRIVETRAIN_ENCODER_BACK_LEFT_VELOCITY_PERCENT = .5;
    public static final double DRIVETRAIN_ENCODER_FRONT_RIGHT_VELOCITY_PERCENT = .5;
    public static final double DRIVETRAIN_ENCODER_BACK_RIGHT_VELOCITY_PERCENT = .5;

    //Encoder Channels
    public static final int DRIVETRAIN_LEFTFRONT_ENCODER_01 = 0;
    public static final int DRIVETRAIN_LEFTFRONT_ENCODER_02 = 1;
    public static final int DRIVETRAIN_LEFTBACK_ENCODER_01 = 2;
    public static final int DRIVETRAIN_LEFTBACK_ENCODER_02 = 3;
    public static final int DRIVETRAIN_RIGHTFRONT_ENCODER_01 = 4;
    public static final int DRIVETRAIN_RIGHTFRONT_ENCODER_02 = 5;
    public static final int DRIVETRAIN_RIGHTBACK_ENCODER_01 = 6;
    public static final int DRIVETRAIN_RIGHTBACK_ENCODER_02 = 7;

    public static final SimpleMotorFeedforward DRIVETRAIN_ENCODER_FEED_FORWARD =
    new SimpleMotorFeedforward(1, 0.8, 0.15); // NEED TO CHANGE
}

public static final class ControllerConstants {
    //Axis values
    public static final int DRIVER_CONTROLLER = 0;
    public static final int DRIVER_CONTROLLER_MOVE_AXISX = 0;
    public static final int DRIVER_CONTROLLER_ROTATE_AXIS = 4;
    public static final int DRIVER_CONTROLLER_MOVE_AXISY = 1;
    
    //Button Mapping
    public static final int DRIVER_CONTROLLER_BUTTON_X = 3;
    public static final int DRIVER_CONTROLLER_BUTTON_B = 2;
    public static final int DRIVER_CONTROLLER_BUTTON_Y = 4;
    public static final int DRIVER_CONTROLLER_BUTTON_A = 1;
    public static final int DRIVER_CONTROLLER_BUMPER_RIGHT = 6;
    public static final int DRIVER_CONTROLLER_BUMPER_LEFT = 5;
}
	
public static final class ShooterConstants {
    //Main Shooter TalonFX
    public static final int SHOOTER_MAIN_TALONFX = 20;
   
}
  
public static final class IntakeConstants {
    //Intake Main SparkMax
    public static final int INTAKE_MAIN_SPARKMAX = 22;  
    //Adding CAN Ports to Solenoid
    public static final int PORT_PCM_MAIN = 2;
}

public static final class FeederConstants {
    //Feeder Motor
    public static final int MAIN_FEEDER_MOTOR = 21;
}

public static final class LEDConstants {
    public static final int LED_SETTING_ONE = 0;
    }
    
public static final class ClimberConstants {
    public static final int DORITO_SOLENOID_ONE = 0;
    public static final int DORITO_SOLENOID_TWO = 0;
    public static final int DORITO_SOLENOID_THREE = 0;
}


}
