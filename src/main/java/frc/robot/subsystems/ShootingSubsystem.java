package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import frc.robot.RobotContainer;
import frc.robot.constants.MechanismConstants;

public class ShootingSubsystem extends SubsystemBase {

    CANSparkMax ShootingControllerRight = new CANSparkMax(MechanismConstants.SHOOTING_RIGHT_CAN_ID, MotorType.kBrushless);
    CANSparkMax IntakeController = new CANSparkMax(MechanismConstants.INTAKE_CAN_ID, MotorType.kBrushed);
    CANSparkMax ShootingControllerLeft = new CANSparkMax(MechanismConstants.SHOOTING_LEFT_CAN_ID, MotorType.kBrushless);
    CANSparkMax MidtakeController = new CANSparkMax(MechanismConstants.MIDTAKE_CAN_ID, MotorType.kBrushless);

    boolean IntakeOn = false;
    boolean wasReleased = true;
    boolean shootingOn = false;
    CommandXboxController rumbleController;
    int rumbleCounter = 0;

    DigitalInput opticalSensor = new DigitalInput(MechanismConstants.SENSOR_DIO_PORT);

    public ShootingSubsystem(CommandXboxController Controller)
    {
        rumbleController = Controller;
    }

    public void shootingPeriodic() {
 
        if(opticalSensor.get() == false && IntakeController.get() == MechanismConstants.INTAKE_SPEED) { // was tripped
            IntakeController.set(0);
            MidtakeController.set(0);
            rumbleController.getHID().setRumble(RumbleType.kBothRumble, 1);
            rumbleCounter = 1;
        }


        if(rumbleCounter == 75)
        {
            rumbleController.getHID().setRumble(RumbleType.kBothRumble, 0);
            rumbleCounter = 0;
        }
        else if(rumbleCounter > 0)
        {
            rumbleCounter += 1;
        }
        
    
        double shootSpeed = ShootingControllerRight.get();
        SmartDashboard.putBoolean("shooting on", shootSpeed != 0);
        boolean shooterAtAmpSpeed = shootSpeed == MechanismConstants.SHOOTING_SPEED_AMP;
        boolean shooterAtSpeakerSpeed = shootSpeed == MechanismConstants.SHOOTING_SPEED_SPEAKER;
        SmartDashboard.putBoolean("at amp speed", shooterAtAmpSpeed);
        SmartDashboard.putBoolean("at speaker speed", shooterAtSpeakerSpeed);
        SmartDashboard.putBoolean("midtake on", MidtakeController.get() != 0);
        SmartDashboard.putBoolean("intake on", IntakeController.get() != 0);
        SmartDashboard.putBoolean("sensor on", !opticalSensor.get());
    }

    public void toggleShootingAmp() {
        if (ShootingControllerRight.get() > 0) {
            ShootingControllerRight.set(0);
            ShootingControllerLeft.set(0);
        } else {
            ShootingControllerRight.set(MechanismConstants.SHOOTING_SPEED_AMP);
            ShootingControllerLeft.set(MechanismConstants.SHOOTING_SPEED_AMP * -1); //normally * -1
        }
    }

    public void toggleShootingSpeaker() {
        if (ShootingControllerRight.get() > 0) {
            ShootingControllerRight.set(0);
            ShootingControllerLeft.set(0);
        } else {
            ShootingControllerRight.set(MechanismConstants.SHOOTING_SPEED_SPEAKER);
            ShootingControllerLeft.set(MechanismConstants.SHOOTING_SPEED_SPEAKER * -1);
        }
    }

    public void toggleMidtake() {
        if (MidtakeController.get() != 0) {
            MidtakeController.set(0);
        } else {
            MidtakeController.set(MechanismConstants.MIDTAKE_SPEED);
        }
    }

    public void toggleMidtakeAndIntake() {
        if(IntakeController.get() != 0) {
            IntakeController.set(0);
            MidtakeController.set(0);
        } else {
            IntakeController.set(MechanismConstants.INTAKE_SPEED);
            MidtakeController.set(MechanismConstants.MIDTAKE_SPEED);
        }

    }

    public void spit()
    {
        IntakeController.set(MechanismConstants.INTAKE_SPEED * -1);
    }

    public void stopSpit()
    {
        if(IntakeController.get() == MechanismConstants.INTAKE_SPEED * -1)
        {
            IntakeController.set(0);
        }
    }

    
}


