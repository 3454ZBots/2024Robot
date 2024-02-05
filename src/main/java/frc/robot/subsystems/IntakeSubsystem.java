package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.MechanismConstants;
public class IntakeSubsystem extends SubsystemBase 
{

    boolean RobotIntake = false;
    boolean IntakeOn = false;
    boolean buttonReleased = false;

    CANSparkMax IntakeController = new CANSparkMax(MechanismConstants.INTAKE_CAN_ID, MotorType.kBrushless);

    public void IntakeOn() 
    {
        IntakeController.set(0.5);

        if (buttonReleased == true)
        {
            IntakeController.set(0);
            buttonReleased = false;

        }
    }

    public void onRelease() 
    {
        buttonReleased = true;  

    }
    
}
