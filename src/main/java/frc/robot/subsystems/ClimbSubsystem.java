package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.MechanismConstants;

public class ClimbSubsystem extends SubsystemBase
{   
    Spark leftClimber = new Spark(MechanismConstants.LEFT_CLIMBER_PORT);
    Spark rightClimber = new Spark(MechanismConstants.RIGHT_CLIMBER_PORT);

    public ClimbSubsystem() { 
        turnOffMotors();
    }
    
    public void turnOffMotors() {
        leftClimber.set(0);
        rightClimber.set(0);
    }

    public void pullDown(){
        leftClimber.set(MechanismConstants.DOWN_CLIMB_SPEED);
        rightClimber.set(MechanismConstants.DOWN_CLIMB_SPEED);
    }

    public void reachUp() {
        leftClimber.set(MechanismConstants.UP_CLIMB_SPEED);
        rightClimber.set(MechanismConstants.UP_CLIMB_SPEED);
    }

}
