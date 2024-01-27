package frc.robot;


import frc.robot.constants.AutoConstants.AutoDriveConstants;
import frc.robot.constants.AutoConstants.AutoTrajectoryConstants;
import frc.robot.constants.BasicConstants.ControllerConstants;
import frc.robot.constants.BasicConstants.SmartDashboardConstants;
import frc.robot.constants.SwerveConstants.SwerveDriveConstants;

import frc.robot.subsystems.DriveSubsystem;
import java.util.List;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

import java.sql.SQLOutput;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.BasicConstants.Misc;

public class RobotContainer {

    private final DriveSubsystem m_robotDrive = new DriveSubsystem();


    // The driver's controller
    CommandXboxController m_driverController = new CommandXboxController(ControllerConstants.DRIVE_REMOTE_PORT);
    //CommandXboxController m_mechanismController = new CommandXboxController(ControllerConstants.MECHANISM_REMOTE_PORT);

    //smart dashboard components
    SendableChooser<Command> m_secondPhaseChooser;
    SendableChooser<Command> m_firstPhaseChooser;

    //autonomous commands
    Command m_driveToGridCommand;
    Command m_placeConeCommand;
    Command m_placeCubeCommand;
    Command m_crossMobilityCommand;
    Command m_straightToChargeStnCommand;
    Command m_driveLeftToChargeStnCommand;
    Command m_driveRightToChargeStnCommand;
    Command m_driveAcrossInnerLine;
    Command m_driveAcrossOuterLine;
    Command m_setXCommand;
    Command m_doNothingCommand;
    Command m_leftTurnCommand;
    Command m_chargeStationCommand;
    Command ArmCmd;
    SequentialCommandGroup m_approachAndPlaceCone;
    SequentialCommandGroup m_approachAndPlaceCube;
    SequentialCommandGroup m_turnRightToChargeStation;
    SequentialCommandGroup m_turnLeftToChargeStation;

    AnalogInput analog;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        m_firstPhaseChooser = new SendableChooser<Command>();
        m_secondPhaseChooser = new SendableChooser<Command>();
        
        // Configure the button bindings and SmartDashboard display
        configureButtonBindings();
        configureAutoCommands();
        configureSmartDashboard();
        analog = new AnalogInput(Misc.sonarPort);
        m_robotDrive.resetEncoders();

        /* 
        Configure default commands
        each subsystem can have one default command, this command is called repeatedly
        useful for things that take joystick input or checking sensors 
        */
        m_robotDrive.setDefaultCommand(
            // The left stick controls translation of the robot.
            // Turning is controlled by the X axis of the right stick.
            // The boolean toggles field oriented driving
            new RunCommand(() ->

                m_robotDrive.drive(
                    m_driverController.getLeftY(),
                    m_driverController.getLeftX(),
                    m_driverController.getRightX(),
                    false),
                m_robotDrive));
        
      
       
    }

    /**
     * Used to define your button->command mappings with {@link m_driverController}.
     */
    private void configureButtonBindings() {

   
        
        
  

        

        m_driverController.y().onTrue(Commands.runOnce(() -> m_robotDrive.setX()));
    }

    /*
     * Command setup to be used for autonomous operations
     */
    private void configureAutoCommands() {
       
        var thetaController =
        new ProfiledPIDController(
            AutoDriveConstants.kPThetaController, 0, 0, AutoDriveConstants.kThetaControllerConstraints);
            thetaController.enableContinuousInput(-Math.PI, Math.PI);

        //first phase commands
        


        m_driveToGridCommand = new SwerveControllerCommand(
            AutoTrajectoryConstants.kDriveToGridTrajectory,
            m_robotDrive::getPose, // Functional interface to feed supplier
            SwerveDriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoDriveConstants.kPXController, 0, 0),
            new PIDController(AutoDriveConstants.kPYController, 0, 0),
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);
        
        m_driveAcrossInnerLine = new SwerveControllerCommand(
            AutoTrajectoryConstants.kDriveAcrossInnerLine,
            m_robotDrive::getPose, // Functional interface to feed supplier
            SwerveDriveConstants.kDriveKinematics,
    
            // Position controllers
            new PIDController(AutoDriveConstants.kPXController, 0, 0),
            new PIDController(AutoDriveConstants.kPYController, 0, 0),
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);

        m_driveAcrossOuterLine = new SwerveControllerCommand(
            AutoTrajectoryConstants.kDriveAcrossOuterLine,
            m_robotDrive::getPose, // Functional interface to feed supplier
            SwerveDriveConstants.kDriveKinematics,
        
            // Position controllers
            new PIDController(AutoDriveConstants.kPXController, 0, 0),
            new PIDController(AutoDriveConstants.kPYController, 0, 0),
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);
 /* 
        m_leftTurnCommand = new SwerveControllerCommand(
                AutoTrajectoryConstants.kDriveForwardTrajectory,
                m_robotDrive::getPose, // Functional interface to feed supplier
                SwerveDriveConstants.kDriveKinematics,
    
                // Position controllers
                new PIDController(AutoDriveConstants.kPXController, 0, 0),
                new PIDController(AutoDriveConstants.kPYController, 0, 0),
                thetaController,
                m_robotDrive::setModuleStates,
                m_robotDrive);
*/
        m_doNothingCommand = new SwerveControllerCommand(
            AutoTrajectoryConstants.kDoNothingTrajectory,
            m_robotDrive::getPose, // Functional interface to feed supplier
            SwerveDriveConstants.kDriveKinematics,            
            // Position controllers
            new PIDController(AutoDriveConstants.kPXController, 0, 0),
            new PIDController(AutoDriveConstants.kPYController, 0, 0),
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);

        m_chargeStationCommand = new SwerveControllerCommand(
            AutoTrajectoryConstants.kChargeStationTrajectory,
            m_robotDrive::getPose, // Functional interface to feed supplier
            SwerveDriveConstants.kDriveKinematics,            
            // Position controllers
            new PIDController(AutoDriveConstants.kPXController, 0, 0),
            new PIDController(AutoDriveConstants.kPYController, 0, 0),
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);

/* 
        m_approachAndPlaceCone = new SequentialCommandGroup(m_driveToGridCommand);
        m_approachAndPlaceCone.addCommands(m_placeConeCommand, m_returnToStartCommand);    
*/
    //    m_approachAndPlaceCube = new SequentialCommandGroup(m_driveToGridCommand);
 //       m_approachAndPlaceCube.addCommands(m_leftTurnCommand);


        //second phase commands
        
    }

    /*
     * Sets up choosers and provides them to the SmartDashboard
     */
    private void configureSmartDashboard() {
    
        //Set options for choosing various flavors of autonomous

        //in the first phase, we drive towards the grid and either place a game piece or continue on
    
        m_firstPhaseChooser.addOption(SmartDashboardConstants.kDepositCube, m_driveToGridCommand);
        m_firstPhaseChooser.addOption(SmartDashboardConstants.kDoNotDeposit, m_doNothingCommand);

        m_secondPhaseChooser.addOption(SmartDashboardConstants.kOuter, m_driveAcrossOuterLine);
        m_secondPhaseChooser.addOption(SmartDashboardConstants.kChargeStation, m_chargeStationCommand);
        m_secondPhaseChooser.addOption(SmartDashboardConstants.kInner, m_driveAcrossInnerLine);
        m_secondPhaseChooser.addOption("Do Nothing", m_doNothingCommand);

        //set default choices        
        m_firstPhaseChooser.setDefaultOption(SmartDashboardConstants.kDepositCube, m_driveToGridCommand);
        m_secondPhaseChooser.setDefaultOption(SmartDashboardConstants.kInner, m_driveAcrossInnerLine);

        //send to SmartDashboard
        SmartDashboard.putData("Scoring: ", m_firstPhaseChooser);
        SmartDashboard.putData("Moving: ", m_secondPhaseChooser);
        
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        /* 
        //Get final composite command
        Command finalCommandGroup = getFinalAutoCommand();

        // Reset odometry to the starting pose of the trajectory.
        m_robotDrive.resetOdometry(AutoTrajectoryConstants.kDoNothingTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return finalCommandGroup.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
        */

      //  m_robotDrive.resetOdometry(AutoTrajectoryConstants.kLeftTurnTrajectory.getInitialPose());

        SequentialCommandGroup finalAutoCommand = new SequentialCommandGroup();

        Command phaseOne = m_firstPhaseChooser.getSelected();
        if (phaseOne != null){
           // finalAutoCommand.addCommands(phaseOne);
        }
        
        Command placeHolderCommand = new WaitCommand(1.0);
       //   finalAutoCommand.addCommands(placeHolderCommand);

        Command phaseTwo = m_secondPhaseChooser.getSelected();
        if (phaseTwo != null) {
           // finalAutoCommand.addCommands(phaseTwo);
        }

        finalAutoCommand.addCommands(m_doNothingCommand, placeHolderCommand, phaseTwo);
        return finalAutoCommand;
    }

    /*
     * Using the input provided by the drive team on the SmartDashboard, compiles the final
     * commands to run during autonomous.
     */

    public void printOutput(){
        SmartDashboard.putNumber("Analog: ", analog.getAverageValue());
        SmartDashboard.putNumber("Left Y: ", m_driverController.getLeftY());
        SmartDashboard.putNumber("Left X: ", m_driverController.getLeftX());
        SmartDashboard.putNumber("Right Y: ", m_driverController.getRightY());
        SmartDashboard.putNumber("Right X: ", m_driverController.getRightX());
        
    }
}
