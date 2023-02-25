package frc;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class Constants {
    //////////////////////////////////////////////////////////////////
    // Drivetrain Physical
    //////////////////////////////////////////////////////////////////
    static public final double WHEEL_BASE_HALF_WIDTH_M = Units.inchesToMeters(23.75/2.0);
    static public final double WHEEL_BASE_HALF_LENGTH_M = Units.inchesToMeters(23.75/2.0);
    static public final double BUMPER_THICKNESS_M = Units.inchesToMeters(2.5);
    static public final double ROBOT_MASS_kg = UnitUtils.lbsToKg(140);
    static public final double ROBOT_MOI_KGM2 = 1.0/12.0 * ROBOT_MASS_kg * Math.pow((WHEEL_BASE_HALF_WIDTH_M*2.2),2) * 2; //Model moment of intertia as a square slab slightly bigger than wheelbase with axis through center

    // See https://www.swervedrivespecialties.com/products/mk4i-swerve-module?variant=39598777172081
    static public final double WHEEL_GEAR_RATIO = 8.14; //L1 gearing
    static public final double AZMTH_GEAR_RATIO = 12.8;
    static public final double WHEEL_FUDGE_FACTOR = 0.9238; // carpet roughtop scrub factor
    static public final double WHEEL_RADIUS_IN = 4.0/2.0 * WHEEL_FUDGE_FACTOR; //four inch diameter wheels - https://www.swervedrivespecialties.com/collections/mk4i-parts/products/billet-wheel-4d-x-1-5w-bearing-bore


    // Drivetrain Performance Mechanical limits
    // Nominal calculations (ideal)
    static private final double MAX_DT_MOTOR_SPEED_RPS = DCMotor.getNEO(1).freeSpeedRadPerSec;
    static private final double MAX_DT_LINEAR_SPEED = MAX_DT_MOTOR_SPEED_RPS / WHEEL_GEAR_RATIO * Units.inchesToMeters(WHEEL_RADIUS_IN);
    // Fudged max expected performance 
    static public final double MAX_FWD_REV_SPEED_MPS = MAX_DT_LINEAR_SPEED * 0.98; //fudge factor due to gearbox losses
    static public final double MAX_STRAFE_SPEED_MPS = MAX_DT_LINEAR_SPEED * 0.98;  //fudge factor due to gearbox losses
    static public final double MAX_ROTATE_SPEED_RAD_PER_SEC = Units.degreesToRadians(360.0); //Fixed at the maximum rotational speed we'd want.
    // Accelerations - also a total guess
    static public final double MAX_TRANSLATE_ACCEL_MPS2 = MAX_FWD_REV_SPEED_MPS/0.50; //0-full time of 0.5 second - this is a guestimate
    static public final double MAX_ROTATE_ACCEL_RAD_PER_SEC_2 = MAX_ROTATE_SPEED_RAD_PER_SEC/.25; //0-full time of 0.25 second - this is a guestaimate


    // Mechanical mounting offsets of the encoder & magnet within the shaft
    // Must be updated whenever the module is reassembled
    // Procedure: 
    // 0 - Put the robot up on blocks.
    // 1 - Reset all these values to 0, deploy code
    // 2 - Pull up dashboard with encoder readings (in radians)
    // 3 - Using a square, twist the modules by hand until they are aligned with the robot's chassis
    // 4 - Read out the encoder readings for each module, put them here
    // 5 - Redeploy code, verify that the  encoder readings are correct as each module is manually rotated
    static public final double FL_ENCODER_MOUNT_OFFSET_RAD = Units.degreesToRadians(143);
    static public final double FR_ENCODER_MOUNT_OFFSET_RAD = Units.degreesToRadians(105);
    static public final double BL_ENCODER_MOUNT_OFFSET_RAD = Units.degreesToRadians(-154);
    static public final double BR_ENCODER_MOUNT_OFFSET_RAD = Units.degreesToRadians(-4);

    //////////////////////////////////////////////////////////////////
    // Vision Processing
    //////////////////////////////////////////////////////////////////

    // Location of vision cameras relative to robot center - currently two in front at 45 degrees, one in back center
    static public final Transform3d robotToFrontRightCameraTrans = new Transform3d(new Translation3d(WHEEL_BASE_HALF_LENGTH_M, -1.0*WHEEL_BASE_HALF_WIDTH_M, 0.25), new Rotation3d(0.0,0.0,-1.0*Math.PI/4.0));
    static public final Transform3d robotToFrontLeftCameraTrans = new Transform3d(new Translation3d(WHEEL_BASE_HALF_LENGTH_M, WHEEL_BASE_HALF_WIDTH_M, 0.25), new Rotation3d(0.0,0.0,Math.PI/4.0));
    static public final Transform3d robotToRearCameraTrans  = new Transform3d(new Translation3d(-1.0*WHEEL_BASE_HALF_LENGTH_M, 0, 0.25), new Rotation3d(0.0,0.0,Math.PI));

    // Vision camera static IP addresses
    static public final String cameraFrontRightIP = "10.17.36.10";
    static public final String cameraFrontLeftIP  = "10.17.36.11";
    static public final String cameraRearIP = "10.17.36.12";

    //////////////////////////////////////////////////////////////////
    // Arm Physical
    //////////////////////////////////////////////////////////////////
    static public final double ARM_BOOM_GEAR_RATIO = 333.33; //gear ratio from motor shaft to boom shaft. Numbers greater than zero indicate a reduction in speed.
    static public final double ARM_STICK_GEAR_RATIO = 274.28; //gear ratio from motor shaft to stick shaft Numbers greater than zero indicate a reduction in speed.
    static public final double ARM_BOOM_MOUNT_HIEGHT = Units.inchesToMeters(40); // Ground to center of boom pivot shaft
    static public final double ARM_BOOM_LENGTH = Units.inchesToMeters(30.25); //Center of boom pivot to center of stick pivot
    static public final double ARM_STICK_LENGTH = Units.inchesToMeters(18.25 + 15.0); //center of stick pivot to far edge of end effector
    static public final double ARM_END_EFF_MAX_VEL_MPS = Units.inchesToMeters(36.0);
    static public final double ARM_END_EFF_MAX_ACCEL_MPS2 = ARM_END_EFF_MAX_VEL_MPS * 2.0;
    static public final double ARM_BOOM_ENCODER_MOUNT_OFFSET_RAD = 0.0;
    static public final double ARM_STICK_ENCODER_MOUNT_OFFSET_RAD = 0.0;
    static public final double ARM_STICK_MAX_ANGLE_DEG = (165);
    static public final double ARM_STICK_MIN_ANGLE_DEG = (-165);
    static public final double ARM_BOOM_MAX_ANGLE_DEG = (70);
    static public final double ARM_BOOM_MIN_ANGLE_DEG = (-140);
    

    //////////////////////////////////////////////////////////////////
    // Electrical
    //////////////////////////////////////////////////////////////////

    // PWM Bank
    public static final int LED_MODE_PORT = 0;
    public static final int LED_DECORATIVE_PORT = 1;
    static public final int CLAW_INTAKE = 2;
    //static public final int UNUSED = 3;
    //static public final int UNUSED = 4;
    //static public final int UNUSED = 5;
    //static public final int UNUSED = 6;
    //static public final int UNUSED = 7;
    //static public final int UNUSED = 8;
    //static public final int UNUSED = 9;

    // DIO Bank
    static public final int FL_AZMTH_ENC_IDX = 0; 
    static public final int FR_AZMTH_ENC_IDX = 1;
    static public final int BL_AZMTH_ENC_IDX = 2;
    static public final int BR_AZMTH_ENC_IDX = 3;
    static public final int ARM_BOOM_ENC_IDX = 4;
    static public final int ARM_STICK_ENC_IDX = 5;
    static public final int FAULT_LED_OUT_IDX = 6;
    static public final int WHITE_LED_OUT_IDX = 7;
    //static public final int UNUSED = 8;
    //static public final int UNUSED = 9;

    // Analog Bank
    static public final int PRESSURE_SENSOR_ANALOG = 0;
    //static public final int UNUSED = 1;
    //static public final int UNUSED = 2;
    //static public final int UNUSED = 3;

    // CAN Bus Addresses - Motors
    //static public final int RESERVED_DO_NOT_USE = 0; // default for most stuff
    //static public final int RESERVED_DO_NOT_USE = 1; // Rev Power Distribution Hub
    static public final int FL_WHEEL_MOTOR_CANID = 2;
    static public final int FL_AZMTH_MOTOR_CANID = 3;
    static public final int FR_WHEEL_MOTOR_CANID = 4;
    static public final int FR_AZMTH_MOTOR_CANID = 5;
    static public final int BL_WHEEL_MOTOR_CANID = 6;
    static public final int BL_AZMTH_MOTOR_CANID = 7;
    static public final int BR_WHEEL_MOTOR_CANID = 8;
    static public final int BR_AZMTH_MOTOR_CANID = 9;
    static public final int ARM_BOOM_MOTOR_CANID = 10;
    static public final int ARM_STICK_MOTOR_CANID = 11;
    static public final int GAMEPIECE_DIST_SENSOR_CANID = 12;
    //static public final int UNUSED = 13;
    //static public final int UNUSED = 14;
    //static public final int UNUSED = 15;
    //static public final int UNUSED = 16;
    //static public final int UNUSED = 17;

    // Pneumatics Hub
    static public final int CLAW_SOLENOID = 0;
    static public final int ARM_BOOM_BRAKE_SOLENOID = 1;
    //static public final int UNUSED = 2;
    //static public final int UNUSED = 3;
    //static public final int UNUSED = 4;
    //static public final int UNUSED = 5;
    //static public final int UNUSED = 6;
    //static public final int UNUSED = 7;
    //static public final int UNUSED = 8;
    //static public final int UNUSED = 9; 

    // PDP Channels - for current measurement
    static public final int CUBE_BLOWER_CURRENT_CHANNEL = 0;
    static public final int BOOM_CURRENT_CHANNEL = 1;
    static public final int STICK_CURRENT_CHANNEL = 2;
	static public final int LEFT_INTAKE_CURRENT_CHANNEL = 3;
    static public final int RIGHT_INTAKE_CURRENT_CHANNEL = 4;
    static public final int FL_WHEEL_CURRENT_CHANNEL = 5;
    static public final int FL_AZMTH_CURRENT_CHANNEL = 6;
    static public final int FR_WHEEL_CURRENT_CHANNEL = 7;
    static public final int FR_AZMTH_CURRENT_CHANNEL = 8;
    static public final int BL_WHEEL_CURRENT_CHANNEL = 9;
    static public final int BL_AZMTH_CURRENT_CHANNEL = 10;
    static public final int BR_WHEEL_CURRENT_CHANNEL = 11;
    static public final int BR_AZMTH_CURRENT_CHANNEL = 12;
    //static public final int UNUSED = 13;
    //static public final int UNUSED = 14;
    //static public final int UNUSED = 15;
    //static public final int UNUSED = 16;
    //static public final int UNUSED = 17;
    //static public final int UNUSED = 18;
    //static public final int UNUSED = 19;
    

    //////////////////////////////////////////////////////////////////
    // Nominal Sample Times
    //////////////////////////////////////////////////////////////////
    public static final double Ts = 0.02;
    public static final double SIM_SAMPLE_RATE_SEC = 0.001;

    //////////////////////////////////////////////////////////////////
    // Field Dimensions
    //////////////////////////////////////////////////////////////////
    static public final double FIELD_WIDTH_M = Units.feetToMeters(27.0);
    static public final double FIELD_LENGTH_M = Units.feetToMeters(54.0);
    static public final Translation2d MAX_ROBOT_TRANSLATION = new Translation2d(FIELD_LENGTH_M, FIELD_WIDTH_M);
    static public final Translation2d MIN_ROBOT_TRANSLATION = new Translation2d(0.0,0.0);
    // Assumed starting location of the robot. Auto routines will pick their own location and update this.
    public static final Pose2d DFLT_START_POSE = new Pose2d(3, 3, new Rotation2d(0));


    /////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////  Derived Constants
    //// - You can reference how these are calculated, but shouldn't
    ////   have to change them
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////

    
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // HELPER ORGANIZATION CONSTANTS
    static public final int FL = 0; // Front Left Module Index
    static public final int FR = 1; // Front Right Module Index
    static public final int BL = 2; // Back Left Module Index
    static public final int BR = 3; // Back Right Module Index
    static public final int NUM_MODULES = 4;

    // Internal objects used to track where the modules are at relative to
    // the center of the robot, and all the implications that spacing has.
    static public final List<Translation2d> robotToModuleTL = Arrays.asList(
        new Translation2d( Constants.WHEEL_BASE_HALF_WIDTH_M,  Constants.WHEEL_BASE_HALF_WIDTH_M),
        new Translation2d( Constants.WHEEL_BASE_HALF_WIDTH_M, -Constants.WHEEL_BASE_HALF_WIDTH_M),
        new Translation2d(-Constants.WHEEL_BASE_HALF_WIDTH_M,  Constants.WHEEL_BASE_HALF_WIDTH_M),
        new Translation2d(-Constants.WHEEL_BASE_HALF_WIDTH_M, -Constants.WHEEL_BASE_HALF_WIDTH_M)
    );

    static public final List<Transform2d> robotToModuleTF = Arrays.asList(
        new Transform2d(robotToModuleTL.get(FL), new Rotation2d(0.0)),
        new Transform2d(robotToModuleTL.get(FR), new Rotation2d(0.0)),
        new Transform2d(robotToModuleTL.get(BL), new Rotation2d(0.0)),
        new Transform2d(robotToModuleTL.get(BR), new Rotation2d(0.0))
    );

    static public final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
        robotToModuleTL.get(FL), 
        robotToModuleTL.get(FR), 
        robotToModuleTL.get(BL), 
        robotToModuleTL.get(BR)
    );
   
}
