package frc.robot.RabidRobot;

public class RabidConstants 
{
		// controller port number in the driver station
		public static final int DRIVE_REMOTE_PORT = 0;
		public static final int MECHANISM_REMOTE_PORT = 1;
		
	
		public static final int MIDTAKE_PORT = 0;


		//CAN IDs for SparkMaxs
		public static final int FRONT_LEFT_ID = 3;
		public static final int FRONT_RIGHT_ID = 5;
		public static final int REAR_LEFT_ID = 4;
		public static final int REAR_RIGHT_ID = 6;
		public static final int SHOOT_ID = 7;
		public static final int INTAKE_PORT1 = 8;
		public static final int INTAKE_PORT2 = 9;
		//Pneumatic Hub CAN ID
		public static final int PNEUMATIC_HUB_ID = 2;
			
		// Pneumatics DoubleSolenoid channels
		// 2020 BC
		public static final int EXTEND_ARM = 0;
		public static final int RETRACT_ARM = 1;
		
		// Controller axis and buttons
		public static final int RIGHT_X = 4;
		public static final int RIGHT_Y = 5;
		
		public static final int LEFT_X = 0;
		public static final int LEFT_Y = 1;
		
		public static final int LEFT_TRIGGER = 2;
		public static final int RIGHT_TRIGGER = 3;
		
		public static final int LEFT_BUMPER = 5;
		public static final int RIGHT_BUMPER = 6;
		
		public static final int BACK = 7;
		public static final int START = 8;

		public static final int BUTTON_Y = 4;
		public static final int BUTTON_A = 1;
		public static final int BUTTON_X = 3;
		public static final int BUTTON_B = 2;
		
		public static final int LEFT_STICK_CLICK = 9;
		public static final int RIGHT_STICK_CLICK = 10;

		//Camera settings 
		public static final int CAM_WIDTH = 180;
		public static final int CAM_HEIGHT = 160;
		public static final int CAMERA_FPS = 15;
		public static final int FRONT_CAM_USB_PORT = 0;
		public static final int BACK_CAM_USB_PORT = 1;

		// Speed constants and general limits
		public static final double DEAD_ZONE = 0.15; // used in Drive class to prevent twitchy controls
	
		// Joystick recorder name constants
		public static final String LYAXIS_NAME = "LyAxis";
		public static final String LXAXIS_NAME = "LxAxis";
		public static final String RYAXIS_NAME = "RyAxis";
		public static final String RXAXIS_NAME = "RxAxis";
		public static final String LTAXIS_NAME = "LTAxis";
		public static final String RTAXIS_NAME = "RTAxis";

		public static final String ABUTTON_NAME = "AButton";
		public static final String BBUTTON_NAME = "BButton";
		public static final String XBUTTON_NAME = "XButton";
		public static final String YBUTTON_NAME = "YButton";
		public static final String LBBUTTON_NAME = "LBButton";
		public static final String RBBUTTON_NAME = "RBButton";

		public static final double KP = -0.1; // proportional constant for limelight
		public static final double RKP = -0.03; // proportional constant for turning for limelight
		public static final double MIN_COMMAND = 0.05; // minimum motor movement for limeligh targeting	
		public static final double LIMELIGHT_HEIGHT = 12;
		public static final double LIMELIGHT_SPEED_FACTOR = 0.35;
		public static final double DISTANCE_IN_INCHES_FROM_TARGET = 180;

		//Shooting Speeds
		public static final double SHORT_SHOOTING_SPEED = -5; //-89.5/127.5;
		public static final double LONG_SHOOTING_SPEED = -5; //-104.5/127.5;

		//Limit Switch Ports
		public static final int UP_SWITCH_PORT = 0;
		public static final int DOWN_SWITCH_PORT = 1;
}
// 600000000 BC (when my gandpa was alive, he could walk uphill both ways)
// 1