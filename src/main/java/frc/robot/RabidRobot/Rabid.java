package frc.robot.RabidRobot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Compressor;
import org.opencv.core.Rect;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.wpilibj.DigitalInput;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.LEDChannel;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.DoubleSolenoid;

// lime light 
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Rabid extends TimedRobot
{
	private Joystick driveRemote;
	//private Joystick mechanismRemote;
	private RabidDrive drive;

	CANSparkMax RRSparkMax ,RLSparkMax, FLSparkMax;
	VictorSPX ShootVic, IntakeVic1, IntakeVic2, FRVic;// ClimbVic;
	VictorSP MidtakeSP;

	NetworkTable table;

	Compressor compressor;

	DigitalInput UpSwitch;

	DoubleSolenoid intakeSolenoid;
	boolean intakeSolenoidBool;

	double leftAxisX, leftAxisY, rightAxisX, rightAxisY, ltAxis;
	int dPadDirectionMechanisms;
	int dPadDirectionDrive;
	boolean aButton, bButton, xButton, yButton, lbButton, rbButton;

	Rect r1, r2;
	boolean frontRectSeen = false, downRectSeen = false;
	double downXCent, frontXCent, frontWidth;

	double xOffset, yOffset;

	double time;

	int roboDirection;

	double goalPose;

	//toggle variables.
	boolean intake;
	boolean intakePressed;
	boolean midtake;
	boolean midtakePressed;

	boolean longShooting;
	boolean longShootingPressed;
	boolean shortShooting;
	boolean shortShootingPressed;

	boolean directionPressed; 

	boolean safeMode;

	boolean orienting;


/* 
	CANifier CanLed;
	int LedSwap;
	double LedA, LedB, LedC;
	*/

	/**
	 * Sets up the robot, setting time to 0, and initializing other mechanisms. 
	 */
	public void robotInit()   
	{
		safeMode = true;
		orienting = false;

		time = 0.0;
		roboDirection = 1;
		goalPose = 1;

		//reference documentation: https://docs.limelightvision.io/en/latest/getting_started.html#basic-programming
		//table = NetworkTableInstance.getDefault().getTable("limelight");
		

	//	CameraServer.startAutomaticCapture();

		UpSwitch = new DigitalInput(RabidConstants.UP_SWITCH_PORT);

		RRSparkMax = new CANSparkMax(RabidConstants.REAR_RIGHT_ID, MotorType.kBrushed);
		RLSparkMax = new CANSparkMax(RabidConstants.REAR_LEFT_ID, MotorType.kBrushed);
		//FRSparkMax = new CANSparkMax(RabidConstants.FRONT_RIGHT_ID, MotorType.kBrushed);
		FRVic = new VictorSPX(RabidConstants.FRONT_RIGHT_ID);
		FLSparkMax = new CANSparkMax(RabidConstants.FRONT_LEFT_ID, MotorType.kBrushed);

		ShootVic = new VictorSPX(RabidConstants.SHOOT_ID);
		//ClimbVic = new VictorSPX(RabidConstants.CLIMB_ID);

		MidtakeSP = new VictorSP(RabidConstants.MIDTAKE_PORT);
		IntakeVic1 = new VictorSPX(RabidConstants.INTAKE_PORT1);
		IntakeVic2= new VictorSPX(RabidConstants.INTAKE_PORT2);

		intakeSolenoid = new DoubleSolenoid(RabidConstants.PNEUMATIC_HUB_ID, PneumaticsModuleType.REVPH, 1, 0);
		intakeSolenoidBool = false;

		compressor = new Compressor(RabidConstants.PNEUMATIC_HUB_ID, PneumaticsModuleType.REVPH);

		driveRemote = new Joystick(RabidConstants.DRIVE_REMOTE_PORT);
		//mechanismRemote = new Joystick(RabidConstants.MECHANISM_REMOTE_PORT);
		drive = new RabidDrive(FLSparkMax, FRVic, RLSparkMax, RRSparkMax);

		// lime light 
		table = NetworkTableInstance.getDefault().getTable("limelight-unique");
		NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
		NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);




		



/* 
		CanLed = new CANifier(6);

		LedA = 0.5;
		LedB = 0;
		LedC = 0;

		LedSwap = 0;
		*/
	}
	
	public void teleopPeriodic() 
	{
		controlRabidDrive();

		if (!safeMode)
		{
			//controlRabidMechanisms();
		}
		else
		{
			compressor.disable();
		}

		SmartDashboard.putBoolean("Intake", intake);
		SmartDashboard.putBoolean("UpSwitch", UpSwitch.get());

		

		//SmartDashboard.putBoolean("Compressor On", compressor.isEnabled());	
		//SmartDashboard.putBoolean("Pneumatic Switch State", compressor.getPressureSwitchValue());
	}

	public void autonomousInit()
	{
		time = 0.0;

		//Reseting Motors in case of field difficulties
		ShootVic.set(ControlMode.PercentOutput, RabidConstants.LONG_SHOOTING_SPEED);
		MidtakeSP.set(0);
		IntakeVic1.set(ControlMode.PercentOutput, 0);
		IntakeVic2.set(ControlMode.PercentOutput, 0);
		drive.mechnumRobot(0, 0, 0, false);

/* 
		CanLed.setLEDOutput(LedA, LEDChannel.LEDChannelA);  //Blue	
		CanLed.setLEDOutput(LedB, LEDChannel.LEDChannelB);	//Red	
		CanLed.setLEDOutput(LedC, LEDChannel.LEDChannelC);	//Green

		//Blue and Yellow Flash Loop
		
		if(LedSwap == 10) {
			if(LedC == 0) {
				LedC = 1;
				LedB = 1;
				LedA = 0;
			}
			else {
				LedC = 0;
				LedB = 0;
				LedA = 0.5;
			}
			LedSwap = 0;
		}
		else {
			LedSwap ++;
		}
*/
	}

	public void autonomousPeriodic()
	{
		if (time < 8.0) 
		{
			ShootVic.set(ControlMode.PercentOutput, RabidConstants.LONG_SHOOTING_SPEED);
			if (time > 6.0) 
			{ 
				MidtakeSP.set(0.75);
			}
		} else if (time < 12.0)
		{
			ShootVic.set(ControlMode.PercentOutput, 0);
			MidtakeSP.set(0);
			drive.mechnumRobot(-0.35, 0.0, 0.0, false);
		} else
		{
			drive.mechnumRobot(0.0, 0.0, 0.0, false);   
		}
		
		time += 0.02; 
	}
/*
	public void controlRabidMechanisms()
	{
		aButton = false;
		xButton = false;

		dPadDirectionMechanisms = mechanismRemote.getPOV();
		yButton = mechanismRemote.getRawButton(RabidConstants.BUTTON_Y);
		bButton = mechanismRemote.getRawButton(RabidConstants.BUTTON_B);
		aButton = mechanismRemote.getRawButton(RabidConstants.BUTTON_A);
		xButton = mechanismRemote.getRawButton(RabidConstants.BUTTON_X);

		SmartDashboard.putBoolean("Intake", intake);
		SmartDashboard.putBoolean("Midtake", midtake);
		SmartDashboard.putBoolean("Long Shot", longShooting);
		SmartDashboard.putBoolean("Short Shot", shortShooting);

		if (bButton)
		{ 
			if (!(longShootingPressed))
			{
				longShooting = !(longShooting);
				longShootingPressed = true;
			}
			
		}
		else
		{
			longShootingPressed = false;
		}

		SmartDashboard.putBoolean("longShooting", longShooting);
		if (yButton)
		{ 
			if (!shortShootingPressed)
			{
				shortShooting = !(shortShooting);
				shortShootingPressed = true;
			}
		}
		else
		{
			shortShootingPressed = false;
		}
		
		if (shortShooting) 
		{
			ShootVic.set(ControlMode.PercentOutput, RabidConstants.SHORT_SHOOTING_SPEED);
			
		}
		else if (longShooting)
		{
			ShootVic.set(ControlMode.PercentOutput, RabidConstants.LONG_SHOOTING_SPEED);
		}
		else
		{	
			ShootVic.set(ControlMode.PercentOutput, 0);
		}

		//intake toggle
		if (xButton)
		{ 
			if (!(intakePressed))
			{
				intake = !(intake);
				intakePressed = true;
			}
		}
		else
		{
			intakePressed = false;
		}
		
		if (intake)
		{
			IntakeVic1.set(ControlMode.PercentOutput, -0.5);
			IntakeVic2.set(ControlMode.PercentOutput, -0.5);
		}
		else 
		{
			IntakeVic1.set(ControlMode.PercentOutput, 0.0);
			IntakeVic2.set(ControlMode.PercentOutput, 0.0);			
		}


		//midtake toggle
		if (aButton)
		{ 
			if (!(midtakePressed))
			{
				midtake = !(midtake);
				midtakePressed = true;
			}
		}
		else 
		{
			midtakePressed = false;
		}

		if (midtake)
		{
			MidtakeSP.set(0.75);
		}
		else 
		{
			MidtakeSP.set(0);			
		}

		//Intake Pneumatics
		if (dPadDirectionMechanisms == 0) {
			intakeSolenoid.set(Value.kForward);
			intakeSolenoidBool = true;
		}
		else if (dPadDirectionMechanisms == 180) {
			intakeSolenoid.set(Value.kReverse);
			intakeSolenoidBool = false;
		}

		//Climbing
		//Assuming that UpSwitch.get() returning true means the switch has not been triggered
		if (dPadDirectionDrive == 0 && UpSwitch.get()) 

		{
			intake = false;
	 		midtake = false;
			longShooting = false;
			shortShooting = false;
			ShootVic.set(ControlMode.PercentOutput, 0);
			IntakeVic1.set(ControlMode.PercentOutput, 0.0);
			IntakeVic2.set(ControlMode.PercentOutput, 0.0);	
			MidtakeSP.set(0);
			//ClimbVic.set(ControlMode.PercentOutput, -0.5);
		}
		else if (dPadDirectionDrive == 180)
		{
			intake = false;
	 		midtake = false;
			longShooting = false;
			shortShooting = false;
			ShootVic.set(ControlMode.PercentOutput, 0);
			IntakeVic1.set(ControlMode.PercentOutput, 0.0);
			IntakeVic2.set(ControlMode.PercentOutput, 0.0);	
			MidtakeSP.set(0);
			//ClimbVic.set(ControlMode.PercentOutput, 1.0);
		}
		else 
		{
			//ClimbVic.set(ControlMode.PercentOutput, 0); 
		}
	}
*/
	public void controlRabidDrive()
	{
		dPadDirectionDrive = driveRemote.getPOV();
		//	if D pad right
		if (dPadDirectionDrive == 90) 
		{
			drive.mechnumRobot(0, 0, -1.0, false);
		}
		else 
		{
			leftAxisX = 0;
			leftAxisY = 0;
			rightAxisX = 0;
			rightAxisY = 0;

			//Retrieving values from the remote (controller)
			leftAxisX = driveRemote.getRawAxis(RabidConstants.LEFT_X);// straifing
			leftAxisY = driveRemote.getRawAxis(RabidConstants.LEFT_Y);// forward/backward
			
			rightAxisX = driveRemote.getRawAxis(RabidConstants.RIGHT_X); // turning
			rightAxisY = driveRemote.getRawAxis(RabidConstants.RIGHT_Y); // unused

			aButton = driveRemote.getRawButton(RabidConstants.BUTTON_A);
			bButton = driveRemote.getRawButton(RabidConstants.BUTTON_B);
			yButton = driveRemote.getRawButton(RabidConstants.BUTTON_Y);


			//reference documentation: https://docs.limelightvision.io/en/latest/getting_started.html#basic-programming

			NetworkTableEntry tx = table.getEntry("tx");
			NetworkTableEntry ty = table.getEntry("ty");
			NetworkTableEntry ta = table.getEntry("ta");

			//X Y Z in meters, Roll Pitch Yaw in degrees
			NetworkTableEntry targetpose = table.getEntry("targetpose_cameraspace");
			

			//read values periodically
			double tX = tx.getDouble(0.0);
			double tY = ty.getDouble(0.0);

			//tag area as percentage of image
			double limeArea = ta.getDouble(0.0);


			double limeZ = targetpose.getDoubleArray(new double[6])[2];
			double limeX = targetpose.getDoubleArray(new double[6])[0];
			double limeY = targetpose.getDoubleArray(new double[6])[1];
			double limeRoll = targetpose.getDoubleArray(new double[6])[3];
			double limePitch = targetpose.getDoubleArray(new double[6])[4];
			double limeYaw = targetpose.getDoubleArray(new double[6])[5];

		
 
			//post to smart dashboard periodically
			SmartDashboard.putNumber("tx", tX);
			SmartDashboard.putNumber("ty", tY);
			SmartDashboard.putNumber("LimelightArea (ta)", limeArea);
			SmartDashboard.putNumber("LimelightZ", limeZ);
			SmartDashboard.putNumber("LimelightX", limeX);
			SmartDashboard.putNumber("LimelightY", limeY);
			SmartDashboard.putNumber("LimelightRoll", limeRoll);
			SmartDashboard.putNumber("LimelightPitch", limePitch);
			SmartDashboard.putNumber("LimelightYaw", limeYaw);
			SmartDashboard.putNumber("distance", estimateDistance());
			

			drive.mechnumRobot(leftAxisY * -1, rightAxisX, leftAxisX * -1, false);


			// limelight distance variables  
			float kpDistance = -0.1f; // proportional control constant for distance
			float current_distance = (float) estimateDistance();
			float desired_distance = 1.1f; // how far you want to be from the april tag
			float kpAim = -0.1f;
			float min_aim_command = 0.05f;
			float distance_error= desired_distance - current_distance;
			float distance_adjust = kpDistance * distance_error;
			SmartDashboard.putNumber("DistanceAdjust", distance_adjust);
			SmartDashboard.putNumber("DistanceError", distance_error);
			double moveInputNum = 0.0;
			float heading_error = (float) (-1.0 * tX);
			float steering_adjust = 0.0f; 
			SmartDashboard.putNumber("Heading Error", heading_error);
			

			if(aButton)
			{
				orienting = true;
			}
			
			if(bButton)
			{
				orienting = false;
			}

			if(orienting)
			{
				if(Math.abs(distance_adjust) > 0.01)
				{
					if (distance_adjust > 0)
					{
						moveInputNum = 0.2;
					}
					else if (distance_adjust < 0)
					{
						moveInputNum = -0.2;
					}
				}

				if (tX > 1.0) {
					steering_adjust = -0.2f;
					//steering_adjust = kpAim * heading_error - min_aim_command;
				} else if (tX < -1.0) {
					steering_adjust = 0.2f;
					//steering_adjust = kpAim * heading_error + min_aim_command;
				}

				if (Math.abs(distance_adjust) <= 0.01 || (tX <= 1.0 && tX >= -1.0)){
					orienting = false;
				}

				SmartDashboard.putNumber("Steering Adjust", steering_adjust);
				drive.mechnumRobot(moveInputNum, steering_adjust, 0, false);
				
				/*
				if(limeRx <= 5 && limeRx >= -5)
				{
					orienting = false;
					drive.mechnumRobot(0, 0, 0, false);
				}
				else if(limeRx > 0)
				{
					drive.mechnumRobot(0, -0.2, 0, false);
				}
				else if(limeRx < 0)
				{
					drive.mechnumRobot(0, 0.2, 0, false);
				}
				*/

			}

			

		}

		SmartDashboard.putNumber("Front Left Drive", drive.getFL());
		SmartDashboard.putNumber("Front Right Drive", drive.getFR());
		SmartDashboard.putNumber("Rear Left Drive", drive.getRL());
		SmartDashboard.putNumber("Rear Right Drive", drive.getRR());
		SmartDashboard.putBoolean("Shooting Mode Active (Y)", roboDirection > 0);
		SmartDashboard.putBoolean("Intake Mode Avtive (Y)", roboDirection < 0);
	}

	public double estimateDistance()
	{
		NetworkTableEntry ty = table.getEntry("ty");
		double targetOffsetAngle_Vertical = ty.getDouble(0.0);
	
		// how many degrees back is your limelight rotated from perfectly vertical?
		double limelightMountAngleDegrees = 25.0; 
	
		// distance from the center of the Limelight lens to the floor
		double limelightLensHeightMeters = 0.485; 
	
		// distance from the target to the floor
		double goalHeightMeters = 1; 
	
		double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
		double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
	
		//calculate distance
		double distanceFromLimelightToGoalMeters = (goalHeightMeters - limelightLensHeightMeters) / Math.tan(angleToGoalRadians);
		//SmartDashboard.putNumber("real distance from math", distanceFromLimelightToGoalMeters);
		return distanceFromLimelightToGoalMeters;
	}
}
//If Janet breaks set @ ~, ~, 0
