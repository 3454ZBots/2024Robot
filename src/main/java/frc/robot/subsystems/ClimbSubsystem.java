package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.MechanismConstants;

public class ClimbSubsystem extends SubsystemBase
{   
    Spark leftClimber = new Spark(MechanismConstants.LEFT_CLIMBER_PORT);
    Spark rightClimber = new Spark(MechanismConstants.RIGHT_CLIMBER_PORT);
    double CLIMB_SPEED = 0.5;
  


    public ClimbSubsystem() { 
        turnOffMotors();
        SmartDashboard.putNumber("Cimb Speed", CLIMB_SPEED);
    }
    
    public void turnOffMotors() {
        leftClimber.set(0);
        rightClimber.set(0);
    }

  

    public void climbLeft()
    {
        leftClimber.set(-CLIMB_SPEED);
    }

    public void stopLeft()
    {
        leftClimber.set(0);
    }
    

    public void climbRight()
    {
        rightClimber.set(CLIMB_SPEED);
    }

    public void stopRight()
    {
        rightClimber.set(0);
    }

    public void toggleClimbing()
    {
        CLIMB_SPEED = CLIMB_SPEED * -1;
     
      
        SmartDashboard.putNumber("Cimb Speed", CLIMB_SPEED);
    }
    /*
    public void leftDown()
    {
        leftClimber.set(MechanismConstants.DOWN_CLIMB_SPEED);
    }

    public void leftUp()
    {
        leftClimber.set(MechanismConstants.UP_CLIMB_SPEED);
    }

    public void RightDown()
    {
        rightClimber.set(MechanismConstants.DOWN_CLIMB_SPEED);
    }

    public void RightUp()
    {
        rightClimber.set(MechanismConstants.UP_CLIMB_SPEED);
    }

    public void leftOff()
    {
        leftClimber.set(0);
    }   

    public void rightOff()
    {
        rightClimber.set(0);
    }

    */

}
