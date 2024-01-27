// OWO
package frc.robot.RabidRobot;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;

public class RabidDrive 
{
	Spark s_frontLeft, s_frontRight, s_rearLeft, s_rearRight;
	double m_frontLeftState, m_frontRightState, m_rearLeftState, m_rearRightState;
	VictorSPX v_frontLeft, v_frontRight, v_rearLeft, v_rearRight;
	CANSparkMax sm_frontLeft, sm_frontRight, sm_rearLeft, sm_rearRight;;
	boolean isSpark; // true if spark false if victor
	boolean isMixed;

	public RabidDrive(CANSparkMax FL, CANSparkMax FR, CANSparkMax RL, CANSparkMax RR)
	{
		sm_frontLeft = FL;
		sm_frontRight = FR;
		sm_rearLeft = RL;
		sm_rearRight = RR;
		isSpark = true;
	}

	public RabidDrive(VictorSPX FL, VictorSPX FR, VictorSPX RL, VictorSPX RR)
	{
		v_frontLeft = FL;
		v_frontRight = FR;
		v_rearLeft = RL;
		v_rearRight = RR;
		isSpark = false;
	}

	public RabidDrive(CANSparkMax FL, VictorSPX FR, CANSparkMax RL, CANSparkMax RR)
	{
		sm_frontLeft = FL;
		v_frontRight = FR;
		sm_rearLeft = RL;
		sm_rearRight = RR;
		isSpark = false;
		isMixed = true;
	}

	public void mechnumRobot(double moveInput, double turnInput, double strafeInput, boolean slowMotionInput)
	{
		double moveSpeed = 0;
		double turnSpeed = 0; //neg = left, pos = right
		double strafeSpeed = 0;

		if (Math.abs(moveInput) > RabidConstants.DEAD_ZONE)
			moveSpeed = moveInput;

		if (Math.abs(turnInput) > RabidConstants.DEAD_ZONE)
			turnSpeed = turnInput;
		
		if (Math.abs(strafeInput) > RabidConstants.DEAD_ZONE)
			strafeSpeed = strafeInput;

		if (moveSpeed != 0 || turnSpeed != 0 || strafeSpeed != 0) {
			double FL = -(moveSpeed - turnSpeed - strafeSpeed);
			double FR = -(moveSpeed + turnSpeed + strafeSpeed);
			double RL = -(moveSpeed - turnSpeed + strafeSpeed);
			double RR = moveSpeed + turnSpeed - strafeSpeed;

			double speedFactorCap = 0.8;
			FL = capSpeed(FL, speedFactorCap);
			FR = capSpeed(FR, speedFactorCap);
			RL = capSpeed(RL, speedFactorCap);
			RR = capSpeed(RR, speedFactorCap);

			if (slowMotionInput) {
				FL *= 0.4;
				FR *= 0.4;
				RL *= 0.4;
				RR *= 0.4;
			}

			if (isSpark) {

				sm_frontLeft.set(FL);
				sm_frontRight.set(FR);
				sm_rearLeft.set(RL); // Why is this different
				sm_rearRight.set(RR);
			}
			else if (isMixed) {
				sm_frontLeft.set(FL);
				v_frontRight.set(ControlMode.PercentOutput, FR);
				sm_rearLeft.set(RL);
				sm_rearRight.set(RR);
			}   
			else {
				v_frontLeft.set(ControlMode.PercentOutput, FL);
				v_frontRight.set(ControlMode.PercentOutput, FR);
				v_rearLeft.set(ControlMode.PercentOutput, RL);
				v_rearRight.set(ControlMode.PercentOutput, RR);
			}

			m_frontLeftState = FL;
			m_frontRightState = FR;
			m_rearLeftState = RL;
			m_rearRightState = RR;
		}
		else {
			if (isSpark) {
			
				sm_frontLeft.set(0);
				sm_frontRight.set(0);
				sm_rearLeft.set(0); // Why is this different
				sm_rearRight.set(0);
			}
			else if (isMixed)
			{
				sm_frontLeft.set(0);
				v_frontRight.set(ControlMode.PercentOutput, 0);
				sm_rearLeft.set(0);
				sm_rearRight.set(0);
			}
			else {
				v_frontLeft.set(ControlMode.PercentOutput, 0);
				v_frontRight.set(ControlMode.PercentOutput, 0);
				v_rearLeft.set(ControlMode.PercentOutput, 0);
				v_rearRight.set(ControlMode.PercentOutput, 0);
			}

			m_frontLeftState = 0;
			m_frontRightState = 0;
			m_rearLeftState = 0;
			m_rearRightState = 0;
		}
	}

	public void sensorDrive(double distance, double xOffset, double yOffset)
	{
		double rotational = 0.0;
		double translate = 0.0;
		double targetDegrees = (Math.atan((102-RabidConstants.LIMELIGHT_HEIGHT)/distance)/ Math.PI) * 180;
		double xerror = -xOffset;

		if (xOffset > 1.0) {
			rotational = (RabidConstants.RKP*xerror - RabidConstants.MIN_COMMAND);
		}
		else if (xOffset < -1.0) {
			rotational = (RabidConstants.RKP*xerror + RabidConstants.MIN_COMMAND);
		}

		double yerror = yOffset;
		System.out.print("Error is:" + yerror);
		if (yOffset < targetDegrees - (0.05 * targetDegrees)) {
			translate = -(RabidConstants.KP*(targetDegrees) - yOffset + RabidConstants.MIN_COMMAND);
		}
		else if (yOffset > targetDegrees + (0.05 * targetDegrees)) {
			translate = -(RabidConstants.KP*(targetDegrees - yOffset) + RabidConstants.MIN_COMMAND);
		}

		double FL = -(translate - rotational);
		double FR = (translate + rotational);
		double RR = -(translate - rotational);
		double RL = (translate + rotational);
		
		FL = capSpeed(FL, RabidConstants.LIMELIGHT_SPEED_FACTOR);
		FR = capSpeed(FR, RabidConstants.LIMELIGHT_SPEED_FACTOR);
		RL = capSpeed(RL, RabidConstants.LIMELIGHT_SPEED_FACTOR);
		RR = capSpeed(RR, RabidConstants.LIMELIGHT_SPEED_FACTOR);

		if (isSpark) {
			/*
			s_frontLeft.set(FL);
			s_frontRight.set(FR);
			s_rearLeft.set(RL);
			s_rearRight.set(RR);
			*/
			v_frontLeft.set(ControlMode.PercentOutput, FL);
			v_frontRight.set(ControlMode.PercentOutput, FR);
			v_rearLeft.set(ControlMode.PercentOutput, RL); // ?
			v_rearRight.set(ControlMode.PercentOutput, RR);
		} 
		else {
			v_frontLeft.set(ControlMode.PercentOutput, FL);
			v_frontRight.set(ControlMode.PercentOutput, FR);
			v_rearLeft.set(ControlMode.PercentOutput, RL);
			v_rearRight.set(ControlMode.PercentOutput, RR);
		}
	}

	public double getFL()
	{
		return m_frontLeftState;
	}

	public double getFR()
	{
		return m_frontRightState;
	}

	public double getRL()
	{
		return m_rearLeftState;
	}

	public double getRR()
	{
		return m_rearRightState;
	}

	private double capSpeed(double speedInput, double maxFactor)
	{
		double absValue = Math.abs(maxFactor);

		if (speedInput >= absValue)
			return absValue;
		else if (speedInput <= (-1 * absValue)) 
			return (-1 * absValue);
		else
			return speedInput;
	}
}
