package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.MechanismConstants;
public class IntakeSubsystem extends SubsystemBase 
{
    

    boolean IntakeOn = false;
    boolean wasReleased = true;

    CANSparkMax IntakeController = new CANSparkMax(MechanismConstants.INTAKE_CAN_ID, MotorType.kBrushless);

    public void IntakePressed() 
    {
        if(wasReleased)
        {
            if(IntakeOn == false)
            {
                IntakeController.set(0.5);
                IntakeOn = true;
            }
            else
            {
                IntakeController.set(0);
                IntakeOn = false;
            }
            wasReleased = false;
        }
    }

    public void IntakeReleased() 
    {
       wasReleased = true;  

    }




}
