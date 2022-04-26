// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

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

    public static final double TIME_DRIVING_IN_AUTON = 1;
    public static final double VELOCITY_DRIVING_IN_AUTON = -0.10;

    public static final class Autonomous {

        public static final double DRIVETRAIN_MAX_SPEED_PER_SECOND_METERS = 3; // NEED TO CHANGE
        public static final double DRIVETRAIN_MAX_ACCELERATION_PER_SECOND_SQUARED_METERS = 3; // NEED TO CHANGE
        public static final double DRIVETRAIN_MAX_ANGULAR_SPEED_PER_SECOND_RADIANS = Math.PI;
        public static final double DRIVETRAIN_MAX_ANGULAR_SPEED_PER_SECOND_SQUARED_RADIANS = Math.PI;

        public static final double DRIVETRAIN_CONTROLLER_PX = 0.05; // NEED TO CHANGE
        public static final double DRIVETRAIN_CONTROLLER_PY = 0.05; // NEED TO CHANGE
        public static final double DRIVETRAIN_CONTROLLER_PTHETA = 0.05; // NEED TO CHANGE

        public static final TrapezoidProfile.Constraints DRIVETRAIN_THETA_CONTROLLER_CONSTRAINTS = 
            new TrapezoidProfile.Constraints(
                DRIVETRAIN_MAX_ANGULAR_SPEED_PER_SECOND_RADIANS, 
                DRIVETRAIN_MAX_ANGULAR_SPEED_PER_SECOND_SQUARED_RADIANS
                );


        public static final class TrajectoryPathWeaver {

            public static final class BasicOne {

                public static final class JsonPath {
                    public static final String TRAJECTORY_PATH_JSON_BASIC_ONE_WP_01 = "paths/Basic1WP1.wpilib.json";
                }
            }

            public static final class AutoCargo {

                public static final class JsonPath {
                    public static final String TRAJECTORY_PATH_JSON_AUTO_CARGO_WP_01 = "paths/AutoCargoWP1.wpilib.json";
                    public static final String TRAJECTORY_PATH_JSON_AUTO_CARGO_WP_02 = "paths/AutoCargoWP2.wpilib.json";
                    public static final String TRAJECTORY_PATH_JSON_AUTO_CARGO_WP_03 = "paths/AutoCargoWP3.wpilib.json";
                }
            }

            public static final class LeftTwo {

                public static final class JsonPath {
                    public static final String TRAJECTORY_PATH_JSON_LEFT_TWO_WP_01 = "paths/left02wp01.json";
                    public static final String TRAJECTORY_PATH_JSON_LEFT_TWO_WP_02 = "paths/left02wp02.json";
                    public static final String TRAJECTORY_PATH_JSON_LEFT_TWO_WP_03 = "paths/left02wp03.json";

                }

            }

            public static final class RightOne {

                public static final class JsonPath {
                    public static final String TRAJECTORY_PATH_JSON_RIGHT_ONE_WP_01 = "paths/right01wp01.json";
                    public static final String TRAJECTORY_PATH_JSON_RIGHT_ONE_WP_02 = "paths/right01wp02.json";
                    public static final String TRAJECTORY_PATH_JSON_RIGHT_ONE_WP_03 = "paths/right01wp03.json";
                }
            }

            public static final class RightTwo {

                public static final class JsonPath {
                    public static final String TRAJECTORY_PATH_JSON_RIGHT_TWO_WP_01 = "paths/right02wp01.json";
                    public static final String TRAJECTORY_PATH_JSON_RIGHT_TWO_WP_02 = "paths/right02wp02.json";
                    public static final String TRAJECTORY_PATH_JSON_RIGHT_TWO_WP_03 = "paths/right02wp03.json";

                }

            }
    
        }
        
    }

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

public static final class Drivetrain {
    //Drivetrain Encoder is Reversed
    public static final boolean DRIVETRAIN_LEFTFRONT_ENCODER_ISREVERSED = true;
    public static final boolean DRIVETRAIN_LEFTBACK_ENCODER_ISREVERSED = true;
    public static final boolean DRIVETRAIN_RIGHTFRONT_ENCODER_ISREVERSED = false;
    public static final boolean DRIVETRAIN_RIGHTBACK_ENCODER_ISREVERSED = false;

    //Drivetrain Encoder distance per pulse
    public static final double DRIVETRAIN_ENCODER_DISTANCE_PER_PULSE = 
    (DrivetrainAutonomousConstants.DRIVETRAIN_WHEEL_DIAMETER_METERS * Math.PI) / (double) EncoderConstants.Drivetrain.DRIVETRAIN_ENCODER_CYCLES_PER_ROTATION;
 
    //Encoder Cycles per Rotation
    public static final int DRIVETRAIN_ENCODER_CYCLES_PER_ROTATION = 2048;
 
    //Encoder Velocity Percent
    public static final double DRIVETRAIN_ENCODER_FRONT_LEFT_VELOCITY_PERCENT = .5;
    public static final double DRIVETRAIN_ENCODER_BACK_LEFT_VELOCITY_PERCENT = .5;
    public static final double DRIVETRAIN_ENCODER_FRONT_RIGHT_VELOCITY_PERCENT = .5;
    public static final double DRIVETRAIN_ENCODER_BACK_RIGHT_VELOCITY_PERCENT = .5;
 
    //Encoder Channels
    public static final int DRIVETRAIN_LEFTFRONT_ENCODER_01 = 10;
    public static final int DRIVETRAIN_LEFTFRONT_ENCODER_02 = 11;
    public static final int DRIVETRAIN_LEFTBACK_ENCODER_01 = 12;
    public static final int DRIVETRAIN_LEFTBACK_ENCODER_02 = 13;
    public static final int DRIVETRAIN_RIGHTFRONT_ENCODER_01 = 14;
    public static final int DRIVETRAIN_RIGHTFRONT_ENCODER_02 = 15;
    public static final int DRIVETRAIN_RIGHTBACK_ENCODER_01 = 16;
    public static final int DRIVETRAIN_RIGHTBACK_ENCODER_02 = 17;
 
    public static final SimpleMotorFeedforward DRIVETRAIN_ENCODER_FEED_FORWARD =
    new SimpleMotorFeedforward(1, 0.8, 0.15); // NEED TO CHANGE

}

public static final class Dorito {
    //Dorito Encoder is Reversed
    public static final boolean DORITO_MOTOR_ENCODER_ISREVERSED = false;

    //Dorito Encoder Distance Per Pulse
    public static final double DORITOMOTOR_DISTANCE_PER_PULSE = 0; //NEED TO CHANGE

    //Cycles Per Revolution
    public static final int DORITOMOTOR_CYCLES_PER_REVOLUTION = 42;

    //Velocity Percent
    public static final double DORITOMOTOR_ENCODER_VELOCITY_PERCENT = .5; //NEED TO CHANGE

    //Dorito Encoder Channels
    public static final int DORITOMOTOR_ENCODER_CHANNEL_01 = 18;
    public static final int DORITOMOTOR_ENCODER_CHANNEL_02 = 19;

    //Dorito Target Position
    public static final double DORITO_TARGET_POS = -40;
}
  
  

   
}

public static final class ControllerConstants {
    //Deadband
    public static final double DRIVER_CONTROLLER_DEADBAND = .15;

    public static final class Driver {
        // Port ID
        public static final int DRIVER_CONTROLLER = 0;

        //Axis values
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
        public static final int DRIVER_CONTROLLER_START_BUTTON = 8;
    }

    public static final class SecondaryDriver {

        // Port ID
        public static final int SECONDARYDRIVER_CONTROLLER = 1;

        //Axis values
        public static final int SECONDARYDRIVER_CONTROLLER_MOVE_AXISX = 0;   
        public static final int SECONDARYDRIVER_CONTROLLER_MOVE_AXISY = 1;   

        //Button Mapping
        public static final int SECONDARYDRIVER_CONTROLLER_BUTTON_RED = 2;
        public static final int SECONDARYDRIVER_CONTROLLER_BUTTON_YELLOW = 4;
        public static final int SECONDARYDRIVER_CONTROLLER_BUTTON_GREEN = 1;
        public static final int SECONDARYDRIVER_CONTROLLER_BUTTON_BLUE = 3;
        
        public static final int SECONDARYDRIVER_CONTROLLER_LEFT_BUMPER = 5;
        public static final int SECONDARYDRIVER_CONTROLLER_RIGHT_BUMPER = 6;
        public static final int SECONDARYDRIVER_CONTROLLER_SWITCH_0 = 5;
        public static final int SECONDARYDRIVER_CONTROLLER_SWITCH_1 = 6;
        public static final int SECONDARYDRIVER_CONTROLLER_SWITCH_2 = 7;
        public static final int SECONDARYDRIVER_CONTROLLER_SWITCH_3 = 8;
    }
   
}
	
public static final class ShooterConstants {
    //CAN Port: Main Shooter TalonFX
    public static final int SHOOTER_MAIN_TALONFX = 20;
    //CAN Port: Hood Motor
    public static final int SHOOTER_HOOD_TALONSRX = 24;
    //CAN Port: Hood Limit Switches
    public static final int SHOOT_HIGH_HOOD_LIMIT = 0;
    public static final int SHOOT_LOW_HOOD_LIMIT = 0;
    //Shooter Default Output
    public static final double SHOOTER_DEFAULT_OUTPUT = 0;
    //ShootHigh Default Output
    public static final double SHOOTHIGH_DEFAULT_OUTPUT = 0.50;
    //ShootLow Default Output
    public static final double SHOOTLOW_DEFAULT_OUTPUT = 0.25;
    //Shooter Hood Default Output
    public static final double SHOOTER_HOOD_DEFAULT_OUTPUT = 0.30;
   
}
  
public static final class IntakeConstants {
    //CAN Port: Intake Main SparkMax
    public static final int INTAKE_MAIN_SPARKMAX = 22;  
    //Default Intake Speed
    public static final double INTAKE_DEFAULT_SPEED = 1.00;
    public static final double INTAKE_DEFAULT_SPEED_REVERSED = -.75;
    
    //Adding CAN Ports to Solenoid
    public static final int PORT_PCM_MAIN = 2;
    //Solenoid Ports on PCM
    public static final int INTAKE_SOLENOIDS_PORT = 4;

}

public static final class FeederConstants {
    //CAN Port: Feeder Motor
    public static final int MAIN_FEEDER_MOTOR = 21;
    //Default Feeder Output
    public static final double FEEDER_DEFAULT_OUTPUT = .75;
    public static final double FEEDER_DEFAULT_OUTPUT_REVERSED = -.75;
}

public static final class LEDConstants {
    //LED Default Setting
    public static final double LED_SETTING_DEFAULT = 0.11;
    public static final double LED_SETTING_CLIMBER_ARM = 0.57;
    public static final double LED_SETTING_CLIMBER_CLAMP_ONE = 0.15;
    public static final double LED_SETTING_CLIMBER_CLAMP_TWO = 0.35;
    public static final double LED_SETTING_CLIMBER_CLAMP_THREE_RED = 0.61;
    public static final double LED_SETTING_CLIMBER_CLAMP_THREE_BLUE = 0.85;

    public static final double LED_SETTING_SHOOT_HIGH = 0.77;
    public static final double LED_SETTING_SHOOT_LOW = 0.85;

    // // Settings
    // public static final double LED_SETTING_SHOOTER = -.69;

    //PWM Port: LED
    public static final int LED_PWM_PORT = 0;
}
    
public static final class ClimberConstants {
    //Clamp Solenoid Ports
    public static final int DORITO_SOLENOID_PORT_ONE = 1;
    public static final int DORITO_SOLENOID_PORT_TWO = 2;
    public static final int DORITO_SOLENOID_PORT_THREE = 3;
    //Lifter Solenoid Port
    public static final int DORITO_LIFTER_SOLENOID_PORT = 0;

    //Dorito Default Speed
    public static final double DORITO_DEFAULT_SPEED = 0.75;
    //Sparkmax CAN port
    public static final int MAIN_DORITO_SPARKMAX = 23;
}


}
