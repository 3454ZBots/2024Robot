package frc.robot.constants;

import frc.robot.constants.SwerveConstants.SwerveDriveConstants;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;




public final class AutoConstants {
    
    /*
     * Basic info about how driving should work with autonomous mode
     * Includes values for all math/physics constants. 
     */
    public static final class AutoDriveConstants
    {
        public static final double kMaxSpeedMetersPerSecond = 0.5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond,
            kMaxAngularSpeedRadiansPerSecondSquared);
    }

    /* 
     * Information about the trajectories to use for various flavors of autonomous
     */
    public static final class AutoTrajectoryConstants
    {
        public static final TrajectoryConfig kAutoTrajectoryConfigForward = new TrajectoryConfig(AutoDriveConstants.kMaxSpeedMetersPerSecond,
            AutoDriveConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(SwerveDriveConstants.kDriveKinematics);
 
 

        public static final Trajectory kDriveToGridTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these interior waypoints, helping to ensure a straight path
                List.of(new Translation2d(0.5, 0)),
                // End 1 meters straight ahead of where we started, facing forward
                new Pose2d(1, 0, new Rotation2d(0)),
                kAutoTrajectoryConfigForward);

     
   }
}
