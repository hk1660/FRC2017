package org.usfirst.frc.team1660.robot;

import edu.wpi.cscore.CvSource;
import edu.wpi.first.wpilibj.CameraServer;
import org.usfirst.frc.team1660.robot.HKdrive;
import org.usfirst.frc.team1660.robot.GripPipeline;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import org.usfirst.frc.team1660.robot.HKdrive;
import com.ctre.CANTalon;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.RobotBase.main;


public class Robot extends SampleRobot {
	AHRS ahrs;
	HKdrive robotDrive;
	NetworkTable table;
	
	//SmartDashboard objects
	SendableChooser startingPosition;
	SendableChooser strategy;
  //DECLARING JOYSTICK VARIABLES   -jamesey
	final int FORWARDBACKWARD_AXIS = 1; //Left joystick up and down
	final int TURNSIDEWAYS_AXIS = 4; //Right joystick side to side
	final int STRAFE_AXIS = 0; //Left joystick side to side
				
	// Channels for the wheels
	final int kFrontLeftChannel = 1;
	final int kRearLeftChannel = 2;
	final int kFrontRightChannel = 4;
	final int kRearRightChannel = 3;
	
	// The channel on the driver station that the joystick is connected to
	final int kJoystickChannel = 0;

	Joystick driverStick = new Joystick(kJoystickChannel);

	public Robot() {
		CANTalon frontLeft = new CANTalon(kFrontLeftChannel);
		CANTalon rearLeft = new CANTalon(kRearLeftChannel);
		CANTalon frontRight = new CANTalon(kFrontRightChannel);
		CANTalon rearRight = new CANTalon(kRearRightChannel);
		
		robotDrive = new HKdrive(frontLeft, rearLeft, frontRight, rearRight);
		robotDrive.setExpiration(0.1);
		
		   try {
				/***********************************************************************
				 * navX-MXP:
				 * - Communication via RoboRIO MXP (SPI, I2C, TTL UART) and USB.            
				 * - See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
				 ************************************************************************/
	            ahrs = new AHRS(SPI.Port.kMXP); 
	        } catch (RuntimeException ex ) {
	            //DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
	        }
	}
	
	
	/* This function is run when the robot is first started up and should be
	  used for any initialization code. */
	public void robotInit() {
		
		table = NetworkTable.getTable("marly");
		// Creates UsbCamera and MjpegServer [1] and connects them

		CameraServer.getInstance().startAutomaticCapture();
		

		// Creates the CvSink and connects it to the UsbCamera 
		//CvSink cvSink = CameraServer.getInstance().getVideo();
		

		// Creates the CvSource and MjpegServer [2] and connects them 
		CvSource outputStream = CameraServer.getInstance().putVideo("steamVideo", 640, 480);
		

		//CHOOSING AUTO MODE
	    startingPosition = new SendableChooser();
        startingPosition.addDefault("Left", new Integer(1));
        startingPosition.addObject("Middle", new Integer(2));
        startingPosition.addObject("Right", new Integer(3));
        SmartDashboard.putData("startingPosition", startingPosition);
        
        strategy = new SendableChooser();
        strategy.addDefault("Move forward only", new Integer(1));

        SmartDashboard.putData("strategy selector", strategy);
		
	}
	

	/* This function is called periodically during autonomous */
	public void autonomous() {
		
	
		 Timer timerAuto = new Timer();
		 timerAuto.start(); 
	//	 int currentStrategy = (int) strategy.getSelected(); 
		 while(isAutonomous() && isEnabled()){ 
			  
			 
			 double timerA = timerAuto.get();
			 SmartDashboard.putNumber("match time",timerA);
		//	   if(currentStrategy == 1) {
			    	runAutoStrategy_GoForwardOnly(timerAuto); 
			//     }  
		 
		 }

	
	}
	
	/* This function is called once each time the robot enters tele-operated mode */
	public void teleopInit() {
		
	}

	/* This function is called periodically during operator control */
	public void teleopPeriodic() {
		
		robotDrive.setSafetyEnabled(true);
		double x = 0;
		double y = 0;
		
		while (isOperatorControl() && isEnabled()) {

			
			checkJoystick();
			
			Timer.delay(0.005); // wait 5ms to avoid hogging CPU cycles

/*		
			Timer.delay(0.25);
			table.putNumber("X", x);
			table.putNumber("Y", y);
			x += 0.05;
			y += 1.0;
*/			

			
		}
	}


	public void operatorControl() {

	}
	
	
	/* This function is called periodically during test mode */
	public void testPeriodic() {
		LiveWindow.run();
	}
	
	
	
	
	/* TELEOP FUNCTIONS */
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
		SmartDashboard.putNumber("angle", ahrs.getAngle() );
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

	

	/* BASE AUTO FUNCTIONS */
	public void goForwardAtSpeed(double speed) {
		robotDrive.mecanumDrive_Cartesian(0, 0, speed, 0);
		
		
		
	}
	
	public void stopDrive() {
		robotDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
	}
	
	public void strafeLeftAtSpeed(double speed) {
		robotDrive.mecanumDrive_Cartesian(-speed, 0, 0, 0);
	}
	
	public void strafeRightAtSpeed(double speed) {
		robotDrive.mecanumDrive_Cartesian(speed, 0, 0, 0);
	}
	
	public void turnLeftAtSpeed(double speed) {
		robotDrive.mecanumDrive_Cartesian(0, speed, 0, 0);
	}
	
	public void turnRightAtSpeed(double speed) {
		robotDrive.mecanumDrive_Cartesian(0, -speed, 0, 0);
	}
	
	public void turnAndStrafe(double strafeSpeed, double turnSpeed) {
		robotDrive.mecanumDrive_Cartesian(-strafeSpeed, -turnSpeed, 0, 0);
	}
	
	public void strategyDestinationPin (Timer timerAuto){

		
	}
	
	public void runAutoStrategy_GoForwardOnly(Timer timerAuto) {
		double timerA = timerAuto.get();
		if(timerA < 2) {
			goForwardAtSpeed(0.3);
		}
		else{
			stopDrive();
		}
	}
	
	
	
	
}