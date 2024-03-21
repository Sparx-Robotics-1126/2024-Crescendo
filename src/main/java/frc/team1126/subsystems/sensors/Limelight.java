package frc.team1126.subsystems.sensors;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Toolbox.AprilTag;
import frc.team1126.Constants.LimelightConstants;

import static frc.team1126.Constants.AprilTags.*;
import static frc.team1126.Constants.LimelightConstants.*;

public class Limelight extends SubsystemBase {
	NetworkTableEntry horizontalOff; // horizontal offset
	NetworkTableEntry validTarget; // whether or not it has valid target
	NetworkTableEntry vetricalOff; // vertical offset

	private double velocity; // velocity
	private double horzontalOffset; // x position
	private double verticalOffset; // y position
	private double accel; // acceleration
	private double timestamp; // timestamp
	private int targetId; // april tag id
	private final NetworkTable table;
	private int angleOnGoalCount;

	public double c_calcResult;
	// private static double distanceToGoal;

	NetworkTableEntry ledMode;
	NetworkTableEntry camMode;
	private static Limelight ll_instance = null;

	public Limelight() {

		table = NetworkTableInstance.getDefault().getTable(LIMELIGHT_TABLE_KEY);

		velocity = 0.0;
		horzontalOffset = 0.0;
		verticalOffset = 0.0;
		accel = 0.0;
		timestamp = 0.0;
		// distanceToGoal = 0.0;
		angleOnGoalCount = 0;
		targetId = -1;

		// 1 if a target is present, or 0 if not.
		validTarget = table.getEntry(HAS_VALID_TARGETS);

		// The X offset of the target in degrees from the crosshair.
		horizontalOff = table.getEntry(HORIZONTAL_OFFSET);

		// The Y offset of the target in degrees from the crosshair.
		vetricalOff = table.getEntry(VERTICAL_OFFSET);
		verticalOffset = vetricalOff.getDouble(0);

		ledMode = table.getEntry(LED_MODE);
		camMode = table.getEntry(CAM_MODE);

		setForTargeting(USE_FOR_TARGETING);
		setLED(LED_ON_DEFAULT);
	}

	public double getAngle() {
		return table.getEntry(HORIZONTAL_OFFSET).getDouble(0.0);
	}

	public void setLimelightPipelineIndex(double idx) {
		LimelightHelpers.setPipelineIndex(LimelightConstants.LIMELIGHT_NAME, idx);
	}

	public int getLimelightPipelineIndex() {
		return (int) LimelightHelpers.getCurrentPipelineIndex(LimelightConstants.LIMELIGHT_NAME);
	}

	public double getCameraHeight() {
		// Add any necessary robot-specific dynamic offsets here (e.g. system elevates
		// the limelight).
		return CAMERA_MIN_FLOOR_HEIGHT;
	}

	public double getCameraPitch() {
		// Add any necessary robot-specific dynamic offsets here (e.g. system tilts the
		// limelight up and down).
		return CAMERA_INITIAL_PITCH;
	}

	public double getXAngle() {
		return getXCrosshairAngle();
	}

	public double getYAngle() {
		return getCameraPitch() + getYCrosshairAngle();
	}

	public double getYOffset(double targetFloorHeight) {
		return Math.abs(targetFloorHeight - getCameraHeight());
	}

	public double getHorizontalDistance(double targetFloorHeight) {
		return getDistance(targetFloorHeight)
				* Math.tan(Math.toRadians(getXAngle()));
	}

	public double getDistance(double targetFloorHeight) {
		return getYOffset(targetFloorHeight)
				/ Math.tan(Math.toRadians(Math.abs(getYAngle())));
	}

	public long getXCrosshairAngle() {
		return horizontalOff.getInteger(0);
	}

	public double getYCrosshairAngle() {
		return vetricalOff.getDouble(0);
	}

	public boolean hasLock() {
		return validTarget.getInteger(0) > 0;
	}

	public void setForTargeting(boolean enable) {
		int camModeNum = enable ? 0 : 1;
		camMode.setNumber(camModeNum);
	}

	public void setLED(boolean enable) {
		int ledModeNum = enable ? 3 : 1;
		ledMode.setNumber(ledModeNum);
	}

	public static Limelight getInstance() {
		if (ll_instance == null) {
			ll_instance = new Limelight();
		}
		return ll_instance;
	}

	public double calculateTargetDistanceInInches() {
		var targetFound = table.getEntry(HAS_VALID_TARGETS).getBoolean(true);
		if (targetId > 0) {
			var tag = getAprilTag(targetId);

			// NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
			// NetworkTableEntry verticalOffset = table.getEntry(VERTICAL_OFFSET);
			// double targetOffsetAngle_Vertical = verticalOffset;

			// how many degrees back is your limelight rotated from perfectly vertical?
			// double limelightMountAngleDegrees = 25.0;

			// distance from the center of the Limelight lens to the floor
			// double limelightLensHeightInches =CAMERA_MIN_FLOOR_HEIGHT;

			double angleToGoalDegrees = getCameraPitch() + verticalOffset;
			double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

			// calculate distance
			return (tag.position.getY() - getCameraHeight()) / Math.tan(angleToGoalRadians);
		}
		return 0;
	}

	private AprilTag getAprilTag(Integer targetId) {

		switch (targetId) {
			case SOURCE_RIGHT_BLUE_ID: {
				return TAG_MAP.get(SOURCE_RIGHT_BLUE_ID);
			}
			case SOURCE_LEFT_BLUE_ID: {
				return TAG_MAP.get(SOURCE_LEFT_BLUE_ID);
			}
			case SOURCE_RIGHT_RED_ID: {
				return TAG_MAP.get(SOURCE_RIGHT_RED_ID);
			}
			case SOURCE_LEFT_RED_ID: {
				return TAG_MAP.get(SOURCE_LEFT_RED_ID);
			}
			case SPEAKER_1_BLUE_ID: {
				return TAG_MAP.get(SPEAKER_1_BLUE_ID);
			}
			case SPEAKER_2_BLUE_ID: {
				return TAG_MAP.get(SPEAKER_2_BLUE_ID);
			}
			case SPEAKER_1_RED_ID: {
				return TAG_MAP.get(SPEAKER_1_RED_ID);
			}
			case SPEAKER_2_RED_ID: {
				return TAG_MAP.get(SPEAKER_2_RED_ID);
			}
			case AMP_BLUE_ID: {
				return TAG_MAP.get(AMP_BLUE_ID);
			}
			case AMP_RED_ID: {
				return TAG_MAP.get(AMP_RED_ID);
			}
			case STAGE_1_BLUE_ID: {
				return TAG_MAP.get(STAGE_1_BLUE_ID);
			}
			case STAGE_2_BLUE_ID: {
				return TAG_MAP.get(STAGE_2_BLUE_ID);
			}
			case STAGE_1_RED_ID: {
				return TAG_MAP.get(STAGE_1_RED_ID);
			}
			case STAGE_2_RED_ID: {
				return TAG_MAP.get(STAGE_2_RED_ID);
			}
			case STAGE_3_BLUE_ID: {
				return TAG_MAP.get(STAGE_3_BLUE_ID);
			}
			case STAGE_3_RED_ID: {
				return TAG_MAP.get(STAGE_3_RED_ID);
			}
			default: {
				return null;
			}
		}
	}

	/**
	 * Gets the already calculated distance from the goal without updating it
	 * 
	 * @return distance
	 */
	public double getDistance() {
		return calculateTargetDistanceInInches();
	}

	public boolean tooClose() {
		return calculateTargetDistanceInInches() <= 20.0;
	}

	public boolean tooFar() {
		return calculateTargetDistanceInInches() >= 115.0;
	}

	public boolean inSpeakerRange(double targetRange) {

		if (hasSpeakerTarget()) {
			if (calculateTargetDistanceInInches() < targetRange + 6
					|| calculateTargetDistanceInInches() > targetRange - 6) {
				return true;
			}
		}
		return false;
	}

	/**
	 * returns if there is a target detected by the limelight
	 */
	public boolean hasTarget() {

		if (targetId > 0) {
			return true;
		}
		
		return false;

		// return validTarget.getBoolean(false);
	}

	public boolean hasSpeakerTarget() {
		if (RobotBase.isReal())
		{
			if ((DriverStation.getAlliance().get() == DriverStation.Alliance.Blue && targetId == SPEAKER_1_BLUE_ID) ||
					(DriverStation.getAlliance().get() == DriverStation.Alliance.Red && targetId == SPEAKER_1_RED_ID)) {
				return true;
			}
		}
		return false;
	}

	public Pose3d getRobotPoseInTargetSpace() {
		if (!hasTarget() || getLimelightPipelineIndex() == LimelightConstants.APRILTAG_PIPELINE)
		 
			return null;
		Pose3d BotPose3d = LimelightHelpers.getBotPose3d_TargetSpace(LimelightConstants.LIMELIGHT_NAME);
		return new Pose3d(BotPose3d.getZ(), -BotPose3d.getX(), BotPose3d.getY(), new Rotation3d(
				BotPose3d.getRotation().getZ(), -BotPose3d.getRotation().getX(), BotPose3d.getRotation().getY()));

	}

	public double calculateTargetAngle() {
		double limelightDistance = getDistance();
		var result = -34 + 1.21 * limelightDistance + -4.7 * Math.pow(10, -3) * Math.pow(limelightDistance, 2);
		c_calcResult = result;
		// System.out.println("Got this result " + result);
		return result;
	}

	@Override
	public void periodic() {
		super.periodic();
		// 1 if a target is present, or 0 if not.
		validTarget = table.getEntry(HAS_VALID_TARGETS);
		velocity = NetworkTableInstance.getDefault().getTable(LIMELIGHT_TABLE_KEY).getEntry(HAS_VALID_TARGETS)
				.getDouble(0);
		horzontalOffset = -1.0 * NetworkTableInstance.getDefault().getTable(LIMELIGHT_TABLE_KEY)
				.getEntry(HORIZONTAL_OFFSET).getDouble(0);
		verticalOffset = NetworkTableInstance.getDefault().getTable(LIMELIGHT_TABLE_KEY).getEntry(VERTICAL_OFFSET)
				.getDouble(0);
		accel = NetworkTableInstance.getDefault().getTable(LIMELIGHT_TABLE_KEY).getEntry(TARGET_AREA).getDouble(0);
		accel = NetworkTableInstance.getDefault().getTable(LIMELIGHT_TABLE_KEY).getEntry(LIMELIGHT_SKEW).getDouble(0);
		targetId = (int) NetworkTableInstance.getDefault().getTable(LIMELIGHT_TABLE_KEY).getEntry("tid").getInteger(0);

		if (Math.abs(horzontalOffset) <= LimelightConstants.VISION_ANGLE_TOLERANCE) {
			angleOnGoalCount++;
		} else {
			angleOnGoalCount = 0;
		}
		SmartDashboard.putNumber("Distance to target", calculateTargetDistanceInInches());
		SmartDashboard.putBoolean("Has Target", hasTarget());
		SmartDashboard.putNumber("Calc Result", calculateTargetAngle());

		c_calcResult = calculateTargetAngle();

	}

	public double getShootingAngle() {
		var closeAngle = 25;
		var midAngle = 34.5;
		var farAngle = 20;
		var ampAngle = 53;

		if (hasSpeakerTarget()) {
			if (calculateTargetDistanceInInches() > 40 && calculateTargetDistanceInInches() < 45) {
				return closeAngle;// close angle
			} else if (calculateTargetDistanceInInches() > 90 && calculateTargetDistanceInInches() < 96) {
				return midAngle;
			} else if (calculateTargetDistanceInInches() > 110 && calculateTargetDistanceInInches() < 114) {
				return farAngle;
			}
		}
		return ampAngle;
	}
}