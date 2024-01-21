package frc.team1126.subsystems.sensors;



import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team1126.Constants.LimelightConstants;
import static frc.team1126.Constants.LimelightConstants.*;

public class Limelight extends SubsystemBase {
	NetworkTableEntry horizontalOff; // horizontal offset
	NetworkTableEntry validTarget; // whether or not it has valid target
	NetworkTableEntry vetricalOff; // vertical offset

	public double velocity; // velocity
	public double horzontalOffset; // x position
	public double verticalOffset; // y position
	public double accel; // acceleration
	public double timestamp; // timestamp
    public int targetId; //april tag id

	private int angleOnGoalCount;
	private static double distanceToGoal;

	NetworkTableEntry ledMode;
	NetworkTableEntry camMode;
	private static Limelight ll_instance = null;
	
	public Limelight() {
		NetworkTable table = NetworkTableInstance.getDefault().getTable(LIMELIGHT_TABLE_KEY);
		
		velocity = 0.0;
		horzontalOffset = 0.0;
		verticalOffset = 0.0;
		accel = 0.0;
		timestamp = 0.0;
		distanceToGoal = 0.0;
		angleOnGoalCount = 0;

		// 1 if a target is present, or 0 if not.
		validTarget = table.getEntry(HAS_VALID_TARGETS);

		// The X offset of the target in degrees from the crosshair.
		horizontalOff = table.getEntry(HORIZONTAL_OFFSET);

		// The Y offset of the target in degrees from the crosshair.
		vetricalOff = table.getEntry(VERTICAL_OFFSET);

		ledMode = table.getEntry(LED_MODE);
		camMode = table.getEntry(CAM_MODE);

		setForTargeting(USE_FOR_TARGETING);
		setLED(LED_ON_DEFAULT);
	}

	public void setLimelightPipelineIndex(int idx) {
		LimelightHelpers.setPipelineIndex(LimelightConstants.LIMELIGHT_NAME, idx);
	  }
	
	  public int getLimelightPipelineIndex() {
		return (int) LimelightHelpers.getCurrentPipelineIndex(LimelightConstants.LIMELIGHT_NAME);
	  }

	public double getCameraHeight() {
		// Add any necessary robot-specific dynamic offsets here (e.g. system elevates the limelight).
		return CAMERA_MIN_FLOOR_HEIGHT;
	}

	public double getCameraPitch() {
		// Add any necessary robot-specific dynamic offsets here (e.g. system tilts the limelight up and down).
		return CAMERA_INITIAL_PITCH;
	}

	public double getXAngle() {
		return getXCrosshairAngle();
	}

	public double getYAngle() {
		return getCameraPitch() + getYCrosshairAngle();
	}

	public double getYOffset(double targetFloorHeight) {
		return Math.abs( targetFloorHeight - getCameraHeight());
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

	public void calculateTargetDistanceInInches() {
        var targetHeight = 0.0;


         // distance from the target to the floor
        switch(targetId)
        {
            case 5:{
               targetHeight = 50.5;
                break;
            }
        }

                    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
                    NetworkTableEntry verticalOffset = table.getEntry(VERTICAL_OFFSET);
                    double targetOffsetAngle_Vertical = verticalOffset.getDouble(0.0);
                
                    // how many degrees back is your limelight rotated from perfectly vertical?
                    double limelightMountAngleDegrees = 25.0; 
                
                    // distance from the center of the Limelight lens to the floor
                    double limelightLensHeightInches = 20.0; 
                
                    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
                    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
                
                    //calculate distance
                    distanceToGoal = (targetHeight - limelightLensHeightInches) / Math.tan(angleToGoalRadians);            
    }

   /**
   * Gets the already calculated distance from the goal without updating it
   * 
   * @return distance
   */
  public double getDistance() {
    return distanceToGoal;
  }

  public static boolean tooClose() {
    return distanceToGoal <= 0.0;
  }

  public static boolean tooFar() {
    return distanceToGoal >= 1.0;
  }

/**
   * returns if there is a target detected by the limelight
   */
  public boolean hasTarget() {
    return validTarget.getBoolean(false);
  }

  public Pose3d getRobotPoseInTargetSpace() {
    if (!hasTarget() || getLimelightPipelineIndex() == LimelightConstants.APRILTAG_PIPELINE)
      return null;
    Pose3d BotPose3d = LimelightHelpers.getBotPose3d_TargetSpace(LimelightConstants.LIMELIGHT_NAME);
    return new Pose3d(BotPose3d.getZ(), -BotPose3d.getX(), BotPose3d.getY(), new Rotation3d(
        BotPose3d.getRotation().getZ(), -BotPose3d.getRotation().getX(), BotPose3d.getRotation().getY()));

  }
  
	@Override
	public void periodic() {
		super.periodic();
		velocity = NetworkTableInstance.getDefault().getTable(LIMELIGHT_TABLE_KEY).getEntry(HAS_VALID_TARGETS).getDouble(0);
		horzontalOffset = -1.0 * NetworkTableInstance.getDefault().getTable(LIMELIGHT_TABLE_KEY).getEntry(HORIZONTAL_OFFSET).getDouble(0);
		verticalOffset = NetworkTableInstance.getDefault().getTable(LIMELIGHT_TABLE_KEY).getEntry(VERTICAL_OFFSET).getDouble(0);
		accel = NetworkTableInstance.getDefault().getTable(LIMELIGHT_TABLE_KEY).getEntry(TARGET_AREA).getDouble(0);
		accel = NetworkTableInstance.getDefault().getTable(LIMELIGHT_TABLE_KEY).getEntry(LIMELIGHT_SKEW).getDouble(0);
        targetId =(int)  NetworkTableInstance.getDefault().getTable(LIMELIGHT_TABLE_KEY).getEntry("tid").getInteger(0);
		calculateTargetDistanceInInches();

		if (Math.abs(horzontalOffset) <= LimelightConstants.VISION_ANGLE_TOLERANCE) {
		  angleOnGoalCount++;
		} else {
		  angleOnGoalCount = 0;
		}
        SmartDashboard.putNumber("Distance to target", distanceToGoal);

	}
}