package org.usfirst.frc.team1111.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Class for the robot to test PID travel distance
 * @author Braidan Duffy
 *
 */
public class Robot extends IterativeRobot {
	double p = .00001, i = 0, d = 0; //TODO: Calibrate
	MiniPID pid = new MiniPID(p, i, d);
	double speed;
	final int TPRE = 4096; //Ticks/Encoder Revolution - Multiply by #GEAR_RATIO to get Ticks/Wheel Revolution
	final int GEAR_RATIO = 8; //Encoder shaft revolutions/wheel shaft revolutions
	final int TPRW = TPRE * GEAR_RATIO; //Ticks/Wheel Revolution
	final int WHEEL_CIRCUMFERENCE = (int) (4 * Math.PI); //Inches
	final int DPT = TPRW / WHEEL_CIRCUMFERENCE; //Ticks/inch
	double desDist = 6; //Inches
	
	TalonSRX frontLeft = new TalonSRX(7); //TODO: Configure
	TalonSRX frontRight = new TalonSRX(43); //TODO: Configure
	TalonSRX backLeft = new TalonSRX(54); //TODO: Configure
	TalonSRX backRight = new TalonSRX(48); //TODO: Configure
	
	TalonSRX motor = new TalonSRX(62);
	
	@Override
	public void robotInit() {
		SmartDashboard.putNumber("P-Gain:", p);
		SmartDashboard.putNumber("I-Gain:", i);
		SmartDashboard.putNumber("D-Gain:", d);
		SmartDashboard.putNumber("Desired Distance:", desDist);
		backRight.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 200);
		pid.setOutputLimits(-1, 1);
	}

	@Override
	public void autonomousInit() {
		p = SmartDashboard.getNumber("P-Gain:", p);
		i = SmartDashboard.getNumber("I-Gain:", i);
		d = SmartDashboard.getNumber("D-Gain:", d);
		pid.setP(p); pid.setI(i); pid.setD(d); //Updates PID values
		resetEncoder();
	}
	
	@Override
	public void autonomousPeriodic() {
		int desTicks = (int) (DPT * desDist);
		SmartDashboard.putNumber("Desired Ticks:", desTicks); //Debug
		pid.setSetpoint(desTicks);
		int ticks = backRight.getSelectedSensorPosition(0);
		SmartDashboard.putNumber("Ticks:", ticks); //Debug
		speed = pid.getOutput(ticks);
		SmartDashboard.putNumber("Speed:", speed); //Debug
		drive(speed);
	}
	
	/**
	 * Drives the robot at set motor speed
	 * @param speed: speed and direction of the drive motors
	 */
	private void drive(double speed) {
		speed *= 1; //Sets soft limit for motors
//		frontLeft.set(ControlMode.PercentOutput, -speed);
//		frontRight.set(ControlMode.PercentOutput, speed);
//		backLeft.set(ControlMode.PercentOutput, -speed);
//		backRight.set(ControlMode.PercentOutput, speed);
		motor.set(ControlMode.PercentOutput, speed);
	}
	
	/**
	 * Wrapper for resetting the encoder. Be sure that this correlates to the designated motor with encoder.
	 */
	private void resetEncoder() {
		backRight.setSelectedSensorPosition(0, 0, 200); //Resets encoder position
	}
}
