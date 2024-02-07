package frc.robot;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.AutoConstants.AutoDriveConstants;
import frc.robot.constants.AutoConstants.AutoTrajectoryConstants;
import frc.robot.constants.SwerveConstants.SwerveDriveConstants;
public class TrajectoryVisualizer extends TimedRobot {
    public void robotInit()
    {
       /*
        TrajectoryConfig kAutoTrajectoryConfigForward = new TrajectoryConfig(AutoDriveConstants.kMaxSpeedMetersPerSecond,
            AutoDriveConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(SwerveDriveConstants.kDriveKinematics);

        Trajectory kDriveAcrossOuterLine =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these interior waypoints, helping to ensure a straight path
                List.of(new Translation2d(1, 0)),
                // End 1 meters straight ahead of where we started, facing forward
                new Pose2d(2, 0, new Rotation2d(0)),
                kAutoTrajectoryConfigForward);
        */


        Field2d m_field = new Field2d();
        SmartDashboard.putData("Field", m_field);
       

        m_field.getObject("traj").setTrajectory(AutoTrajectoryConstants.kChargeStationTrajectory);
        //m_field.setRobotPose(robotPose);
    }
}
