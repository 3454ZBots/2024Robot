package frc.robot.subsystems;
// lime light 
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionSubsystem {

    NetworkTable table; 

    public VisionSubsystem() {
	    table = NetworkTableInstance.getDefault().getTable("limelight-unique");
	    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
	    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
    }

    public void getData() {
        //reference documentation: https://docs.limelightvision.io/en/latest/getting_started.html#basic-programming

		NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");
        NetworkTableEntry ta = table.getEntry("ta");

        //X Y Z in meters, Roll Pitch Yaw in degrees
        NetworkTableEntry targetpose = table.getEntry("targetpose_cameraspace");

        //read values periodically
        double tX = tx.getDouble(0.0);
        double tY = ty.getDouble(0.0);

        //tag area as percentage of image
        double limeArea = ta.getDouble(0.0);

        double limeZ = targetpose.getDoubleArray(new double[6])[2];
        double limeX = targetpose.getDoubleArray(new double[6])[0];
        double limeY = targetpose.getDoubleArray(new double[6])[1];
        double limeRoll = targetpose.getDoubleArray(new double[6])[3];
        double limePitch = targetpose.getDoubleArray(new double[6])[4];
        double limeYaw = targetpose.getDoubleArray(new double[6])[5];

        //post to smart dashboard periodically
        SmartDashboard.putNumber("tx", tX);
        SmartDashboard.putNumber("ty", tY);
        SmartDashboard.putNumber("LimelightArea (ta)", limeArea);
        SmartDashboard.putNumber("LimelightZ", limeZ);
        SmartDashboard.putNumber("LimelightX", limeX);
        SmartDashboard.putNumber("LimelightY", limeY);
        SmartDashboard.putNumber("LimelightRoll", limeRoll);
        SmartDashboard.putNumber("LimelightPitch", limePitch);
        SmartDashboard.putNumber("LimelightYaw", limeYaw);
        SmartDashboard.putNumber("distance", estimateDistance());

        // limelight distance variables  
        float kpDistance = -0.1f; // proportional control constant for distance
        float current_distance = (float) estimateDistance();
        float desired_distance = 1.1f; // how far you want to be from the april tag
        float kpAim = -0.1f;
        float min_aim_command = 0.05f;
        float distance_error= desired_distance - current_distance;
        float distance_adjust = kpDistance * distance_error;
        SmartDashboard.putNumber("DistanceAdjust", distance_adjust);
        SmartDashboard.putNumber("DistanceError", distance_error);
        double moveInputNum = 0.0;
        float heading_error = (float) (-1.0 * tX);
        float steering_adjust = 0.0f; 
		SmartDashboard.putNumber("Heading Error", heading_error);
			 
        if (tX > 1.0) {
            steering_adjust = kpAim * heading_error - min_aim_command;
        } else if (tX < -1.0) {
            steering_adjust = kpAim * heading_error + min_aim_command;
        }
    }

    public double estimateDistance()
	{
		NetworkTableEntry ty = table.getEntry("ty");
		double targetOffsetAngle_Vertical = ty.getDouble(0.0);
	
		// how many degrees back is your limelight rotated from perfectly vertical?
		double limelightMountAngleDegrees = 25.0; 
	
		// distance from the center of the Limelight lens to the floor
		double limelightLensHeightMeters = 0.485; 
	
		// distance from the target to the floor
		double goalHeightMeters = 1; 
	
		double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
		double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
	
		//calculate distance
		double distanceFromLimelightToGoalMeters = (goalHeightMeters - limelightLensHeightMeters) / Math.tan(angleToGoalRadians);
		//SmartDashboard.putNumber("real distance from math", distanceFromLimelightToGoalMeters);
		return distanceFromLimelightToGoalMeters;
	}

}