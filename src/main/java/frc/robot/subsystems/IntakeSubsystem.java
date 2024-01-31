package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
public class IntakeSubsystem extends SubsystemBase

{
    CANSparkMax IntakeController = new CANSparkMax(0, MotorType.kBrushless);

    public void IntakeOn() {

        IntakeController.set(0.5);

    }

    public void IntakeOff() {

        IntakeController.set(0);

    }
}
