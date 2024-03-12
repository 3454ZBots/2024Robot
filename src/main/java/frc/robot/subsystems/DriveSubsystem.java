package frc.robot.subsystems;
import javax.xml.datatype.DatatypeConstants.Field;

import org.ejml.simple.SimpleMatrix;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.Publisher;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SwerveConstants.SwerveDriveConstants;


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
    private Field2d m_field = new Field2d();
    private Field2d fakefield = new Field2d();


    boolean isFieldOriented = true;
    boolean released = true;

    
    Matrix<N3,N1> poseDeviations = VecBuilder.fill(0.1, 0.1, 0.1);
    Matrix<N3,N1> visionDeviations = VecBuilder.fill(0.9, 0.9, 0.9);

    StructArrayPublisher<SwerveModuleState> publisher = NetworkTableInstance.getDefault().getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();
    
    SwerveDriveOdometry m_odometry;

        SwerveDrivePoseEstimator m_PoseEstimator;
    /** Creates a new DriveSubsystem. */
    public DriveSubsystem() {
        m_gyro.setYaw(0);

        m_odometry = new SwerveDriveOdometry(
        SwerveDriveConstants.kDriveKinematics,
        getHeading(),
        getModulePositions());


        m_PoseEstimator = new SwerveDrivePoseEstimator(SwerveDriveConstants.kDriveKinematics, m_gyro.getRotation2d(), getModulePositions(), getPose(), poseDeviations, visionDeviations);
        
        SmartDashboard.putData("field pose", m_field);
        SmartDashboard.putData("Rotation 2D Pose", fakefield);
    }
    
     
    @Override
    public void periodic() 
    {
        // Update the odometry in the periodic block
            m_odometry.update(
            getHeading(),
            new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_rearLeft.getPosition(),
                m_rearRight.getPosition()
            });
            m_PoseEstimator.update(m_gyro.getRotation2d(), getModulePositions());
            m_field.setRobotPose(m_PoseEstimator.getEstimatedPosition());


            SwerveModuleState[] states = new SwerveModuleState[]
            {
                m_frontLeft.getState(),
                m_frontRight.getState(),
                m_rearLeft.getState(),
                m_rearRight.getState()
            };
            
            SmartDashboard.putNumber("FL Distance", m_frontLeft.getPosition().distanceMeters);
            SmartDashboard.putNumber("FR Distance", m_frontRight.getPosition().distanceMeters);
            SmartDashboard.putNumber("RL Distance", m_rearLeft.getPosition().distanceMeters);
            SmartDashboard.putNumber("RR Distance", m_rearRight.getPosition().distanceMeters);
            

        publisher.set(states);

        SmartDashboard.putNumber("FL Speed", m_frontLeft.getState().speedMetersPerSecond);
        SmartDashboard.putNumber("FR Speed", m_frontRight.getState().speedMetersPerSecond);
        SmartDashboard.putNumber("RL Speed", m_rearLeft.getState().speedMetersPerSecond);
        SmartDashboard.putNumber("RR Speed", m_rearRight.getState().speedMetersPerSecond);

        
        fakefield.setRobotPose(new Pose2d(0, 0, m_gyro.getRotation2d()));
    }

    @Override
    public void simulationPeriodic ()
    {
        m_PoseEstimator.update(Rotation2d.fromDegrees(m_gyro.getAngle() * -1), getModulePositions());
        m_field.setRobotPose(getPose());

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
            getHeading(),
            new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_rearLeft.getPosition(),
                m_rearRight.getPosition()
            },
            pose);
    }

    public void toggleFieldOriented(boolean pressed)
    {
       
     
        SmartDashboard.putBoolean("Field-Oriented", isFieldOriented);

        if(pressed && released)
        {
            isFieldOriented = !isFieldOriented;
            released = false;
        }
        if(!pressed)
        {
            released = true;
        }
    
    }


    public void drivePeriodic(double controllerLY, double controllerLX, double controllerRX)
    {  
            drive(controllerLY, controllerLX, controllerRX, isFieldOriented);
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
        SmartDashboard.putNumber("IMU Heading:", getHeading().getDegrees());
        SmartDashboard.putNumber("IMU Turn Rate", getTurnRate());

        
        SwerveModuleState[] swerveModuleStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative ? 
            ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getHeading()) : 
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

     
    public Rotation2d getHeading() {
        return m_gyro.getRotation2d();
    }




    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */

     
    public double getTurnRate() {
        return m_gyro.getRate() * (SwerveDriveConstants.kGyroReversed ? -1.0 : 1.0);
    }
    private SwerveModulePosition[] getModulePositions() 
    {
        return  new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_rearLeft.getPosition(),
                m_rearRight.getPosition()
        };
    }


    public void driveDistance(double dist)
    {
        m_frontLeft.encoderDrive(dist);
        m_frontRight.encoderDrive(dist);
        m_rearLeft.encoderDrive(dist);
        m_rearRight.encoderDrive(dist);
    }
    
    //Commands we added
}