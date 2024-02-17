package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import frc.robot.constants.MechanismConstants;

public class ShootingSubsystem extends SubsystemBase {

    CANSparkMax ShootingControllerRight = new CANSparkMax(MechanismConstants.SHOOTING_RIGHT_CAN_ID, MotorType.kBrushless);
    CANSparkMax IntakeController = new CANSparkMax(MechanismConstants.INTAKE_CAN_ID, MotorType.kBrushed);
    CANSparkMax ShootingControllerLeft = new CANSparkMax(MechanismConstants.SHOOTING_LEFT_CAN_ID, MotorType.kBrushless);
    CANSparkMax MidtakeController = new CANSparkMax(MechanismConstants.MIDTAKE_CAN_ID, MotorType.kBrushless);

    boolean IntakeOn = false;
    boolean wasReleased = true;
    boolean shootingOn = false;

    DigitalInput opticalSensor = new DigitalInput(MechanismConstants.SENSOR_DIO_PORT);

    public void shootingPeriodic() {
        if(opticalSensor.get() == true) { // was tripped
            IntakeController.set(0);
            MidtakeController.set(0);
        }
        SmartDashboard.putBoolean("shooting on", ShootingControllerRight.get() > 0);
        SmartDashboard.putBoolean("midtake on", MidtakeController.get() > 0);
        SmartDashboard.putBoolean("intake on", IntakeController.get() > 0);
        SmartDashboard.putBoolean("sensor on", opticalSensor.get());
    }

    public void toggleShooting() {
        if (ShootingControllerRight.get() > 0) {
            ShootingControllerRight.set(0);
            ShootingControllerLeft.set(0);
        } else {
            ShootingControllerRight.set(MechanismConstants.SHOOTING_SPEED);
            ShootingControllerLeft.set(MechanismConstants.SHOOTING_SPEED);
        }
    }

    public void toggleMidtake() {
        if (MidtakeController.get() > 0) {
            MidtakeController.set(0);
        } else {
            MidtakeController.set(MechanismConstants.MIDTAKE_SPEED);
        }
    }

    public void toggleMidtakeAndIntake() {
        if(IntakeController.get() > 0) {
            IntakeController.set(0);
            MidtakeController.set(0);
        } else {
            IntakeController.set(MechanismConstants.INTAKE_SPEED);
            MidtakeController.set(MechanismConstants.MIDTAKE_SPEED);
        }
    }
}
