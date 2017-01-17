package org.usfirst.frc.team1660.robot;

import edu.wpi.first.wpilibj.Joystick;

import org.usfirst.frc.team1660.robot.HKdrive;

import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;
//import.edu.wpi.first.wpilibj.networktables.NetworkTable;


import com.ctre.CANTalon;
//import com.kauailabs.navx;



/**
 * This is a demo program showing how to use Mecanum control with the RobotDrive
 * class.
 */
public class Robot extends SampleRobot {
	HKdrive robotDrive;
	NetworkTable table;
	
	  //DECLARING JOYSTICK VARIABLES   -jamesey
		int FORWARDBACKWARD_AXIS = 1; //Left joystick up and down
		int TURNSIDEWAYS_AXIS = 4; //Right joystick side to side
		int STRAFE_AXIS = 0; //Left joystick side to side
		
		int LIFTDROP_AXIS = 1; //Left joystick up and down
		
	// Channels for the wheels
	final int kFrontLeftChannel = 2;
	final int kRearLeftChannel = 3;
	final int kFrontRightChannel = 1;
	final int kRearRightChannel = 0;
	
	// The channel on the driver station that the joystick is connected to
	final int kJoystickChannel = 0;

	Joystick driverStick = new Joystick(kJoystickChannel);

	public Robot() {
		CANTalon rearLeft = new CANTalon(kRearLeftChannel);
		CANTalon rearRight = new CANTalon(kRearRightChannel);
		CANTalon frontLeft = new CANTalon(kFrontLeftChannel);
		CANTalon frontRight = new CANTalon(kFrontRightChannel);
		robotDrive = new HKdrive(frontLeft, rearLeft, frontRight, rearRight);
		robotDrive.setInvertedMotor(MotorType.kFrontLeft, true); // invert the
																	// left side
																	// motors
		robotDrive.setInvertedMotor(MotorType.kRearLeft, true); // you may need
																// to change or
																// remove this
																// to match your
																// robot
		robotDrive.setExpiration(0.1);
	}

	public void robotInit() {
		
		table = NetworkTable.getTable("marly");
		
	}
	
	public void autonomous(){
		
	}
	
	
	/**
	 * Runs the motors with Mecanum drive.
	 */
	@Override
	public void operatorControl() {
		robotDrive.setSafetyEnabled(true);
		double x = 0;
		double y = 0;
		
		while (isOperatorControl() && isEnabled()) {

			
			checkJoystick();
			
			Timer.delay(0.25);
			table.putNumber("X", x);
			table.putNumber("Y", y);
			x += 0.05;
			y += 1.0;
			
			Timer.delay(0.005); // wait 5ms to avoid hogging CPU cycles
		}
	}
	
	
	//MOVE DRIVETRAIN WITH XBOX360 JOYSTICKS -Matthew
	public void checkJoystick()
	{
		
		 double threshold = 0.11;
		 
		 double strafe = squareInput(driverStick.getRawAxis(STRAFE_AXIS)) ; // right and left on the left thumb stick?
		 double moveValue = squareInput(driverStick.getRawAxis(FORWARDBACKWARD_AXIS));// up and down on left thumb stick?
		 double rotateValue = squareInput(driverStick.getRawAxis(TURNSIDEWAYS_AXIS));// right and left on right thumb stick
		
		 //KILL GHOST MOTORS -Matthew & Dianne
		if(moveValue > threshold*-1 && moveValue < threshold) {
			moveValue = 0;
		}
		if(rotateValue > threshold*-1 && rotateValue < threshold) {
			rotateValue = 0;
		}
		if(strafe > threshold*-1 && strafe < threshold) {
			strafe = 0;
		}
		
		//MECANUM -Matthew
		SmartDashboard.putNumber(  "move",        moveValue);
		SmartDashboard.putNumber(  "rotate",        rotateValue);
		SmartDashboard.putNumber(  "Strafe",        strafe);
	    
		robotDrive.mecanumDrive_Cartesian( strafe, -rotateValue, -moveValue, 0); //imu.getYaw()
		
	}

	public double squareInput(double x) {
        if(x > 0 ) {
      	return Math.pow(x, 4);
        }
        else{
      	  return -1*Math.pow(x, 4); 
        }
      }

	
	
	
	
	
}
