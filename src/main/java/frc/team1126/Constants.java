package frc.team1126;

// import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.Toolbox.AprilTag;
import frc.lib.Toolbox.PIDGains;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.PIDConstants;

import frc.lib.swervelib.parser.PIDFConfig;

public final class Constants {
    public static final boolean DEFAULT_CONSTANT_PROPERTIES = RobotBase.isReal();
    public static final double NOMINAL_VOLTAGE = 12;
    public static final int MAX_CURRENT = 80;
    public static final double DEAD_BAND = .5;

 public static final class AutonConstants
  {

    public static final PIDConstants TRANSLATION_PID = new PIDConstants(2, 0, 0);
    public static final PIDConstants ANGLE_PID   = new PIDConstants(2.5, 0, 0.9);
    public static final double MAX_MODULE_SPEED = 4.5;
  }


    public static final class Auton
    {

        public static final PIDFConfig X_AUTO_PID     = new PIDFConfig(0.7, 0, 0);
        public static final PIDFConfig Y_AUTO_PID     = new PIDFConfig(0.7, 0, 0);
        public static final PIDFConfig ANGLE_AUTO_PID = new PIDFConfig(0.4, 0, 0.01);

        public static final double MAX_SPEED        = 2;
        public static final double MAX_ACCELERATION = 2;
    }

    public static class AprilTags {
        public static final int SOURCE_RIGHT_BLUE_ID = 1;
        public static final int SOURCE_LEFT_BLUE_ID = 2;
        public static final int SOURCE_RIGHT_RED_ID = 9;
        public static final int SOURCE_LEFT_RED_ID = 10;
        public static final int SPEAKER_1_RED_ID = 4;
        public static final int SPEAKER_1_BLUE_ID = 7;
        public static final int SPEAKER_2_RED_ID = 3;
        public static final int SPEAKER_2_BLUE_ID = 8;
        public static final int AMP_RED_ID = 5;
        public static final int AMP_BLUE_ID = 6;
        public static final int STAGE_1_RED_ID = 12;
        public static final int STAGE_2_RED_ID = 11;
        public static final int STAGE_3_RED_ID = 13;
        public static final int STAGE_1_BLUE_ID = 15;
        public static final int STAGE_2_BLUE_ID = 14;
        public static final int STAGE_3_BLUE_ID = 16;
        public AprilTag[] tags = new AprilTag[16];
        public static final HashMap<Integer, AprilTag> TAG_MAP = new HashMap<Integer, AprilTag>() {
            {
                put(SOURCE_RIGHT_BLUE_ID, new AprilTag() {
                    {
                        id = SOURCE_RIGHT_BLUE_ID;
                        name = "Source Right Blue";
                        position = new Pose2d(new Translation2d(19.375, 48.125), Rotation2d.fromDegrees(0.0));
                    }
                });
                put(SOURCE_LEFT_BLUE_ID, new AprilTag() {
                    {
                        id = SOURCE_LEFT_BLUE_ID;
                        name = "Source Left Blue";
                        position = new Pose2d(new Translation2d(-19.375, 48.125), Rotation2d.fromDegrees(0.0));
                    }
                });
                put(SOURCE_RIGHT_RED_ID, new AprilTag() {
                    {
                        id = SOURCE_RIGHT_RED_ID;
                        name = "Source Right Red";
                        position = new Pose2d(new Translation2d(19.375, 48.125), Rotation2d.fromRadians(0.0));
                    }
                });
                put(SOURCE_LEFT_RED_ID, new AprilTag() {
                    {
                        id = SOURCE_LEFT_RED_ID;
                        name = "Source Left Red";
                        position = new Pose2d(new Translation2d(-19.375, 48.125), Rotation2d.fromRadians(0.0));
                    }
                });
                put(SPEAKER_1_RED_ID, new AprilTag() {
                    {
                        id = SPEAKER_1_RED_ID;
                        name = "Speaker 1 Red";
                        position = new Pose2d(new Translation2d(0,56.875), Rotation2d.fromRadians(0.0));
                    }
                });
                put(SPEAKER_1_BLUE_ID, new AprilTag() {
                    {
                        id = SPEAKER_1_BLUE_ID;
                        name = "Speaker 1 Blue";
                        position = new Pose2d(new Translation2d(0,56.875), Rotation2d.fromDegrees(0.0));
                    }
                });
                put(SPEAKER_2_RED_ID, new AprilTag() {
                    {
                        id = SPEAKER_2_RED_ID;
                        name = "Speaker 2 Red";
                        position = new Pose2d(new Translation2d(17, 56.875), Rotation2d.fromRadians(0.0));
                    }
                });
                put(SPEAKER_2_BLUE_ID, new AprilTag() {
                    {
                        id = SPEAKER_2_BLUE_ID;
                        name = "Speaker 2 Blue";
                        position = new Pose2d(new Translation2d(17, 56.875), Rotation2d.fromDegrees(0.0));
                    }
                });
                put(AMP_RED_ID, new AprilTag() {
                    {
                        id = AMP_RED_ID;
                        name = "Amp Red";
                        position = new Pose2d(new Translation2d(0, 48.125), Rotation2d.fromRadians(0.0));
                    }
                });
                put(AMP_BLUE_ID, new AprilTag() {
                    {
                        id = AMP_BLUE_ID;
                        name = "Amp Blue";
                        position = new Pose2d(new Translation2d(0, 48.125), Rotation2d.fromDegrees(0.0));
                    }
                });
                put(STAGE_1_RED_ID, new AprilTag() {
                    {
                        id = STAGE_1_RED_ID;
                        name = "Stage 1 Red";
                        position = new Pose2d(new Translation2d(0, 47.5), Rotation2d.fromRadians(0.0));
                    }
                });
                put(STAGE_2_RED_ID, new AprilTag() {
                    {
                        id = STAGE_2_RED_ID;
                        name = "Stage 2 Red";
                        position = new Pose2d(new Translation2d(0, 47.5), Rotation2d.fromRadians(0.0));
                    }
                });
                put(STAGE_3_RED_ID, new AprilTag() {
                    {
                        id = STAGE_3_RED_ID;
                        name = "Stage 3 Red";
                        position = new Pose2d(new Translation2d(0, 47.5), Rotation2d.fromRadians(0.0));
                    }
                });
                put(STAGE_1_BLUE_ID, new AprilTag() {
                    {
                        id = STAGE_1_BLUE_ID;
                        name = "Stage 1 Blue";
                        position = new Pose2d(new Translation2d(0, 47.5), Rotation2d.fromDegrees(0.0));
                    }
                });
                put(STAGE_2_BLUE_ID, new AprilTag() {
                    {
                        id = STAGE_2_BLUE_ID;
                        name = "Stage 2 Blue";
                        position = new Pose2d(new Translation2d(0, 47.5), Rotation2d.fromDegrees(0.0));
                    }
                });
                put(STAGE_3_BLUE_ID, new AprilTag() {
                    {
                        id = STAGE_3_BLUE_ID;
                        name = "Stage 3 Blue";
                        position = new Pose2d(new Translation2d(0, 47.5), Rotation2d.fromDegrees(0.0));
                    }
                });
            }
        };
    }

    public static class CANdleConstants {
        /* CANdle ID */
        public static final int CANDLE_ID = 15;

        /* Purple RGB */
        public static final int PURPLE_R = 255;
        public static final int PURPLE_G = 0;
        public static final int PURPLE_B = 191;

        /* Yellow RGB */
        public static final int YELLOW_R = 255;
        public static final int YELLOW_G = 130;
        public static final int YELLOW_B = 0;

        /* Red RGB */
        public static final int RED_R = 255;
        public static final int RED_G = 0;
        public static final int RED_B = 0;

        /* Green RGB */
        public static final int GREEN_R = 0;
        public static final int GREEN_G = 255;
        public static final int GREEN_B = 0;

        /* Blue RGB */
        public static final int BLUE_R = 0;
        public static final int BLUE_G = 0;
        public static final int BLUE_B = 255;

    }

    public static class LimelightConstants {
        public static final boolean USE_FOR_TARGETING = true;
        public static final boolean LED_ON_DEFAULT = false;
        public static final double CAMERA_MIN_FLOOR_HEIGHT = 8.628;
        public static final double CAMERA_INITIAL_PITCH = 33.5;
        // declare ID's of pipelines here
        public static final int K_CUBE_PIPELINE = 0;
        public static final int K_REFLECTIVE_PIPELINE = 1;
        public static final int K_APRILTAG_PIPELINE = 4;

        // PID values for limelight
        public static final PIDGains K_LL_TARGET_GAINS = new PIDGains(0.008, 0, 0);

        public static final PIDGains K_LL_PUPPY_GAINS = new PIDGains(0.02, 0, 0); // .008
        public static final PIDGains K_LL_PUPPY_DRIVE_GAINS = new PIDGains(0.008, 0, 0);
        public static final double K_PUPPY_TURN_MOTION_SMOOTHING = 0.3;
        public static final double K_PUPPY_DRIVE_MOTION_SMOOTHING = 0.4;

        public static final PIDGains K_LL_ALIGN_STRAFE_GAINS = new PIDGains(.04, 0.0015, 0.001);
        public static final PIDGains K_LL_ALIGN_DRIVE_GAINS = new PIDGains(.025, 0.0015, 0.0005);
        public static final double K_ALIGN_DRIVE_MOTION_SMOOTHING = 0;
        public static final double K_ALIGN_STRAFE_MOTION_SMOOTHING = 0;

        public static final double SPEAKER_HEIGHT = 78.0; // height needs to be changed
        public static final double AMP_HEIGHT = 26.0;
        public static final double LIMELIGHT_HEIGHT = 44; // measured from bottom of bumper to bottom of limelight
        public static final double INITIAL_ANGLE = 0;
        public static final double VISION_ANGLE_TOLERANCE = 0;
        public static final double APRILTAG_PIPELINE = 0;
        public static final String LIMELIGHT_NAME = "";

        public static final String LIMELIGHT_TABLE_KEY = "limelight";
        public static final String HORIZONTAL_OFFSET = "tx";
        public static final String HAS_VALID_TARGETS = "tv";
        public static final String VERTICAL_OFFSET = "ty";
        public static final String TARGET_AREA = "ta";
        public static final String LIMELIGHT_SKEW = "ts";
        public static final String LED_MODE = "ledMode";
        public static final String CAM_MODE = "camMode";
    }

    /* Field related constants */
    public static final class FieldConstants {
        // List of possible scoring locations as Pose2d objects
        public static final List<Pose2d> SCORING_POSITIONS =
                List.of(
                        new Pose2d(
                                new Translation2d(0.555, 7.436),
                                Rotation2d.fromRadians(Math.PI)), // Red loading double station
                        new Pose2d(new Translation2d(0.555, 6.146), Rotation2d.fromRadians(Math.PI)),
                        new Pose2d(
                                new Translation2d(15.03, 5.061),
                                Rotation2d.fromDegrees(0.0)), // Red node scoring locations
                        new Pose2d(new Translation2d(15.03, 4.405), Rotation2d.fromDegrees(0.0)),
                        new Pose2d(new Translation2d(15.03, 3.846), Rotation2d.fromDegrees(0.0)),
                        new Pose2d(new Translation2d(15.03, 3.298), Rotation2d.fromDegrees(0.0)),
                        new Pose2d(new Translation2d(15.03, 2.74), Rotation2d.fromDegrees(0.0)),
                        new Pose2d(new Translation2d(15.03, 2.2), Rotation2d.fromDegrees(0.0)),
                        new Pose2d(new Translation2d(15.03, 1.62), Rotation2d.fromDegrees(0.0)),
                        new Pose2d(new Translation2d(15.03, 1.06), Rotation2d.fromDegrees(0.0)),
                        new Pose2d(new Translation2d(15.03, 0.52), Rotation2d.fromDegrees(0.0)),
                        new Pose2d(
                                new Translation2d(15.64, 7.430),
                                Rotation2d.fromDegrees(0.0)), // Blue loading double substation
                        new Pose2d(new Translation2d(15.64, 6.16), Rotation2d.fromDegrees(0.0)),
                        new Pose2d(
                                new Translation2d(1.598, 4.996),
                                Rotation2d.fromRadians(-Math.PI)), // Blue node scoring locations
                        new Pose2d(new Translation2d(1.598, 4.373), Rotation2d.fromRadians(-Math.PI)),
                        new Pose2d(new Translation2d(1.598, 3.85), Rotation2d.fromRadians(-Math.PI)),
                        new Pose2d(new Translation2d(1.598, 3.3), Rotation2d.fromRadians(-Math.PI)),
                        new Pose2d(new Translation2d(1.598, 2.75), Rotation2d.fromRadians(-Math.PI)),
                        new Pose2d(new Translation2d(1.598, 2.2), Rotation2d.fromRadians(-Math.PI)),
                        new Pose2d(new Translation2d(1.598, 1.63), Rotation2d.fromRadians(-Math.PI)),
                        new Pose2d(new Translation2d(1.598, 1.05), Rotation2d.fromRadians(-Math.PI)),
                        new Pose2d(new Translation2d(1.598, 0.5), Rotation2d.fromRadians(-Math.PI)));
    }


    /**
     * General robot constants
     */
    public static final class GeneralConstants {

        // Driver controller port
        public static final int DRIVER_CONTROLLER_ID = 0;

        // Operator controller port
        public static final int OPERATOR_CONTROLLER_ID = 1;

        public static final double CLOSE_SPEAKER_ANGLE = 22.5;

        public static final double MID_SPEAKER_ANGLE = 41;

        public static final double AMP_ANGLE = 50;

        public static final double DRIVE_ANGLE = 34.5;

        public static final double CLOSE_SPEAKER_POWER = 2800;

        public static final double MID_SPEAKER_POWER = 3800;

        public static final double STORAGE_POWER = 0.7;

        public static final double LOW_STORAGE_POWER = 0.6;
    }

    /**
     * Constants revolving around swerve subsystem
     */
    public static class SwerveConstants {

        // Joystick axis deadband for the swerve drive

        public static final double SWERVE_DEADBAND = 0.1;

        // Swerve default translational scalar
        public static final double SWERVE_NORMAL_TRANSLATION = 0.6;

        // Swerve slow translational scalar
        public static final double SWERVE_SLOW_TRANSLATION = 0.25;

        // Hold time on motor brakes when disabled
        public static final double WHEEL_LOCK_TIME = 10;

        public static final boolean IS_FIRST_ORDER = true;

        public static final double DT_CONSTANT = 0.1;

        public static final boolean HEADING_CORRECTION = false;

        public static final boolean CHASSIS_VELOCITY_CORRECTION = false;

        //for endgame rumble feature
        public static final int ENDGAME_SECONDS = 30;
        public static final int STOP_RUMBLE_SECONDS = 28;
    }

    public static class ClimberConstants {
        
        public static final double K_GEAR_RATIO = 10.71;// 10.75; // 8.45;// 10.71;
        public static final double K_WHEEL_DIAMETER_INCHES = 6;
        public static final double K_WHEEL_DIAMETER_METERS = Units.inchesToMeters(K_WHEEL_DIAMETER_INCHES);
        public static final double kEncoderDistanceConversionFactor = ((Math.PI * K_WHEEL_DIAMETER_METERS) / (K_GEAR_RATIO));

        public static final int MOTOR_LEFT_ID = 20;
        public static final int MOTOR_RIGHT_ID = 21;

        public static final double MAX_HEIGHT = 4;
        public static final int LEFT_DIGITAL_INPUT = 10;
        public static final int RIGHT_DIGITAL_INPUT = 11;
    }

    public static class StorageConstants {
        
        public static final int ACQ_WHEELS_ID = 25;
        public static final int LIGHT_SENSOR = 15; // what the light sensor id was for 2023 extension, probably DIO 
    }

    public static class ShooterConstants {
        public static final int SHOOTER_ID = 26;
    }

    public static class ArmConstants {
        public static final int MASTER_ID = 30;
        public static final int SLAVE_ID = 31;
        public static final int ARM_PIGEON_ID = 16;
        public static final int ARM_LIMIT_SWITCH_ID = 19;
        public static final double kArmP = 15.0; // 6.4
        public static final double kArmI = 0.00;
        public static final double kArmD = 0.00;
        public static final double kArmFF = 0.00;
        public static final int kCurrentLimitDefault = 20;
        public static final int kCurrentLimitManual = 10;
        public static final int kCurrentLimitClimbing = 70;
        public static final double kArmMinOutput = -1.0; // -1.00
        public static final double kArmMaxOutput = 1.0; // 1.00
        public static final double kPositionTolerance = 0.05;
        public static final double kVelocityTolerance = 0.02;
    }

    /**
     * Constants revolving around the vision subsystem.
     */
    public static final class VisionConstants {
        // Camera name
        public static final String CAMERA_NAME = "OV5647";

        // Robot to camera transform
        public static final Transform3d ROBOT_TO_CAM =
                new Transform3d(
                        new Translation3d(0.0, Units.inchesToMeters(1.5), Units.inchesToMeters(39.0)),
                        new Rotation3d(0.0, 0.0, 0.0));
    }

    /**
     * Constants revolving around auton modes.
     */
    // public static final class AutonConstants {

    //     public static final double MAX_VELOCITY = 3.0;
    //     public static final double MAX_ACCELERATION = 2.0;
    //     public static final PathConstraints CONSTRAINTS =
    //             new PathConstraints(AutonConstants.MAX_VELOCITY, AutonConstants.MAX_ACCELERATION);

    //     public static final double XY_CONTROLLER_P = 4;
    //     public static final double THETA_CONTROLLER_P = 1;
    // }
}
  
