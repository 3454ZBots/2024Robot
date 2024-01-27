package frc.robot.subsystems;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SwerveConstants;
import frc.robot.constants.BasicConstants.ControllerConstants;
import frc.robot.constants.SwerveConstants.NeoMotorConstants;
import frc.robot.constants.SwerveConstants.SwerveDriveConstants;
import com.ctre.phoenix6.hardware.Pigeon2;


/**
 * Sets up swerve modules for the robot.
 */
public class DriveSubsystem extends SubsystemBase {
    // Create MAXSwerveModules
    private final SwerveModule m_frontLeft = new SwerveModule(
        SwerveDriveConstants.kFrontLeftDrivingCanId,
        SwerveDriveConstants.kFrontLeftTurningCanId,
        SwerveDriveConstants.kFrontLeftChassisAngularOffset);

    private final SwerveModule m_frontRight = new SwerveModule(
        SwerveDriveConstants.kFrontRightDrivingCanId,
        SwerveDriveConstants.kFrontRightTurningCanId,
        SwerveDriveConstants.kFrontRightChassisAngularOffset);

    private final SwerveModule m_rearLeft = new SwerveModule(
        SwerveDriveConstants.kRearLeftDrivingCanId,
        SwerveDriveConstants.kRearLeftTurningCanId,
        SwerveDriveConstants.kBackLeftChassisAngularOffset);

    private final SwerveModule m_rearRight = new SwerveModule(
        SwerveDriveConstants.kRearRightDrivingCanId,
        SwerveDriveConstants.kRearRightTurningCanId,
        SwerveDriveConstants.kBackRightChassisAngularOffset);

    
    // The gyro sensor
    private Pigeon2 m_gyro = new Pigeon2(SwerveDriveConstants.PIGEON_ID);


    // Odometry class for tracking robot pose
    SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
        SwerveDriveConstants.kDriveKinematics,
        Rotation2d.fromDegrees(getHeading()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

    /** Creates a new DriveSubsystem. */
    public DriveSubsystem() {
        m_gyro.setYaw(0);
    }

     
    @Override
    public void periodic() {
        // Update the odometry in the periodic block
            m_odometry.update(
            Rotation2d.fromDegrees(getHeading()),
            new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_rearLeft.getPosition(),
                m_rearRight.getPosition()
            });
    
  }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    
    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(
            Rotation2d.fromDegrees(getHeading()),
            new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_rearLeft.getPosition(),
                m_rearRight.getPosition()
            },
            pose);
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        // Adjust input based on max speed
        
        if (Math.abs(xSpeed) < 0.1) 
        {
            xSpeed = 0;
        }

        if (Math.abs(ySpeed) < 0.1) 
        {
            ySpeed = 0;
        }

        if (Math.abs(rot) < 0.1) 
        {
            rot = 0;
        }
        

        xSpeed *= SwerveDriveConstants.kMaxSpeedMetersPerSecond;
        ySpeed *= SwerveDriveConstants.kMaxSpeedMetersPerSecond;
        rot *= SwerveDriveConstants.kMaxAngularSpeed;

        SmartDashboard.putNumber("Rot", rot);
        SmartDashboard.putNumber("IMU Heading:", getHeading());
        SmartDashboard.putNumber("IMU Turn Rate", getTurnRate());

        // to do - replace radions:60 with new getHeading
        SwerveModuleState[] swerveModuleStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative ? 
            ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(getHeading())) : 
            new ChassisSpeeds(xSpeed, ySpeed, rot));

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveDriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_rearLeft.setDesiredState(swerveModuleStates[2]);
        m_rearRight.setDesiredState(swerveModuleStates[3]);
    } 

    /**
     * Sets the wheels into an X formation to prevent movement.
     */
    public void setX() {

        SwerveModuleState test = new SwerveModuleState(0, Rotation2d.fromDegrees(45));
        m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
        m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
        m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
        m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
        
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
            desiredStates, SwerveDriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_rearLeft.setDesiredState(desiredStates[2]);
        m_rearRight.setDesiredState(desiredStates[3]);
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        m_frontLeft.resetEncoders();
        m_rearLeft.resetEncoders();
        m_frontRight.resetEncoders();
        m_rearRight.resetEncoders();
    }

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        m_gyro.reset();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */

     
    public double getHeading() {
        return Rotation2d.fromDegrees(m_gyro.getAngle() * -1).getDegrees();
    }




    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */

     
    public double getTurnRate() {
        return m_gyro.getRate() * (SwerveDriveConstants.kGyroReversed ? -1.0 : 1.0);
    }

    
    //Commands we added
}