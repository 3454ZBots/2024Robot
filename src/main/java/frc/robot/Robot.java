package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.BasicConstants.ControllerConstants;
import frc.robot.subsystems.DriveSubsystem;
//import frc.robot.subsystems.pixy.I2CLink;
//import frc.robot.subsystems.pixy.PixyCamera;
import frc.robot.constants.BasicConstants.ArmConstants;

public class Robot extends TimedRobot {

    // PROPERTIES & FIELDS

    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;
   // private CameraServer cameraServer;
  //  private PixyCamera pixy;


    //private CANSparkMax shoulderSparkMax;
    //private DoubleSolenoid clawSolenoid;

   // boolean aButton, bButton, clawClosed, aChange;


    //Joystick Values
    //double leftAxisX, rightAxisX, rightAxisY, leftAxisY;

    //private XboxController mechanismController;

    //METHODS

    /*
     * Initializes the robot, run when the robot is first started.
     * Subcomponent intialization is abstracted.
     */
    @Override
    public void robotInit() {
        m_robotContainer = new RobotContainer();
       // CameraServer.startAutomaticCapture();
      //  pixy = new PixyCamera(new I2CLink());

     //   mechanismController = new XboxController(1);

  
    }

    /*
    * Runs every ~20ms, in teleop, autonomous, and disabled states.
      Helpful for polling sensor states, running diagnostics, etc.
     */
    @Override
    public void robotPeriodic()
    { 
        CommandScheduler.getInstance().run();
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
    @Override
    public void autonomousInit() {
        /*
        An example of how we can retrieve selections from the SmartDashboard 
        to run a specific "mode" of autonomous. May be useful to select starting "slot",
        whether we're going to place a game piece, and whether we're going to try to get 
        on the ramp.

        I'd recommend having a SmartDashboard constants class to handle all these string
        values instead of hardcoding things here.

        See ref: https://docs.wpilib.org/en/stable/docs/software/dashboards/smartdashboard/choosing-an-autonomous-program-from-smartdashboard.html#command-based
        */

        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
    var test = CommandScheduler.getInstance();
    }

    @Override
    public void teleopInit() {
        
        // This makes sure that the autonomous stops running when
        // teleop starts running.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }



    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {

        m_robotContainer.printOutput();
     //   pixy.run();



       

    }


  


    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {

    }

}
