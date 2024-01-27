package frc.robot.constants;

/* Contains constants that are relevant to every robot,
 * regardless of yearly challenge, drive system, or other subsystems
 */
public final class BasicConstants 
{
	/* Constants related to the driver controls for the robot. */
	public static final class ControllerConstants
	{
		// controller port number in the driver station
		public static final int DRIVE_REMOTE_PORT = 0;
		public static final int MECHANISM_REMOTE_PORT = 1;

		// Controller axis and buttons

		//analog controls (provide values in a range of values, not just on/off)
		public static final int RIGHT_X = 4;
		public static final int RIGHT_Y = 5;
		
		public static final int LEFT_X = 0;
		public static final int LEFT_Y = 1;
		
		public static final int LEFT_TRIGGER = 2;
		public static final int RIGHT_TRIGGER = 3;
		
		public static final int LEFT_BUMPER = 5;
		public static final int RIGHT_BUMPER = 6;
		
		//buttons (on/off foutputs)
		public static final int BACK = 7;
		public static final int START = 8;

		public static final int BUTTON_Y = 4;
		public static final int BUTTON_A = 1;
		public static final int BUTTON_X = 3;
		public static final int BUTTON_B = 2;
		
		public static final int LEFT_STICK_CLICK = 9;
		public static final int RIGHT_STICK_CLICK = 10;

		public static final double DEAD_ZONE = 0.15;
	}

	/* Information about constants for use on the SmartDashboard */
	public static final class SmartDashboardConstants
	{
        public static final String kDepositCube = "Deposit Cube";
        public static final String kDoNotDeposit = "Do not deposit";

		public static final String kOuter = "Outer side";
		public static final String kChargeStation = "Charge station";
		public static final String kInner = "Inner side";

	}

	public static final class Misc 
	{
		public static final int sonarPort = 0; 
	}

	/**
	 * Constants for the movement speed of motors in the arm, and other stuff like that.
	 */
	public static final class ArmConstants
	{
		public static final int shoulderMotorCanID = 10;
		public static final int pneumaticHubCanID = 12;
		public static final int extensionPWM = 1;
		public static final int wristCanID = 11;
		public static final double armkP = 0;
		public static final double armkI = 0;
		public static final double armkD = 0;
		public static final int clawMotorPWM = 0;
		public static final int outSwitchDIO = 0;
		public static final int inSwitchDIO = 1;
	
	}
}