package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.constants.MechanismConstants;
public class IntakeSubsystem extends SubsystemBase

{
    CANSparkMax IntakeController = new CANSparkMax(MechanismConstants.INTAKE_CAN_ID, MotorType.kBrushless);

    public void IntakeOn() {

        IntakeController.set(MechanismConstants.INTAKE_SPEED);

    }

    public void IntakeOff() {

        IntakeController.set(0);

    }
}
