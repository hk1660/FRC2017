package org.usfirst.frc.team1660.robot;


import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import org.usfirst.frc.team1660.robot.GripPipeline;
import org.usfirst.frc.team1660.robot.HKdrive;
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
import com.ctre.CANTalon;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.vision.VisionRunner;
import edu.wpi.first.wpilibj.vision.VisionThread;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;



public class Robot extends SampleRobot {
	AHRS ahrs;
	HKdrive robotDrive;
	AnalogInput ultraSonic = new AnalogInput(0);
	NetworkTable table;
	private  VisionThread visionThread;
	
	//SmartDashboard objects
	SendableChooser startingPosition;
	SendableChooser strategy;
	DigitalInput limitSwitch = new DigitalInput(0);
	
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
	
	//values for coordinates of the peg, the robot sees
	int target1x;
	int target1y;
	int target2x;
	int target2y;
	int finalPegx;
	int finalPegy;
	int pegX;
	int pegY;
	
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
		
		// limit switch object
		
		
		
		
		
		NetworkTable.setIPAddress("10.16.60.63");
		table = NetworkTable.getTable("marly");
		
		// Creates UsbCamera and MjpegServer [1] and connects them
		UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
		
		// Creates the CvSource and MjpegServer [2] and connects them 
		CvSource outputStream = CameraServer.getInstance().putVideo("steamVideo", 640, 480);
		
		// Creates the CvSink and connects it to the UsbCamera 
		CvSink cvSink = CameraServer.getInstance().getVideo();
		
		//pipeline.process(camera);
		
	    visionThread = new VisionThread(camera, new GripPipeline(), pipeline -> {
	        if (!pipeline.filterContoursOutput().isEmpty()) {
	  
	        	Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput ().get(0));
	        	SmartDashboard.putString("CamRec X", r.x );
	            //System.out.println(pipeline.filterContoursOutput().get(0));
	            //SmartDashboard.putNumber("opencv",pipeline.filterContoursOutput().get(0));
	            table = NetworkTable.getTable("GRIP/marly");
	        
	            double[] def = new double[0];
	            double[] areas = table.getNumberArray("width", def);
	            System.out.println(areas[0]);
	        }
	    });
	    visionThread.start();


		//CHOOSING AUTO MODE
	    startingPosition = new SendableChooser();
        startingPosition.addDefault("Left", new Integer(1));
        startingPosition.addObject("Middle", new Integer(2));
        startingPosition.addObject("Right", new Integer(3));
        SmartDashboard.putData("startingPosition", startingPosition);
        
        strategy = new SendableChooser();
        strategy.addDefault("Move forward only", new Integer(1));
 
        SmartDashboard.putData("strategy selector", strategy);
		//limitSwitch.addobject(limt)
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


	public void operatorControl() {
		System.out.println("operatorControl");
		robotDrive.setSafetyEnabled(true);
		double x = 0;
		double y = 0;
		
		while (isOperatorControl() && isEnabled()) {
			 //System.out.println("operatorControl2");
	          //Timer.delay(0.020);		/* wait for one motor update time period (50Hz)     */
	          
	          checkJoystick();
	          //checkGyro();
	          getDistance();
			
			Timer.delay(0.005); // wait 5ms to avoid hogging CPU cycles

		
			//table.putNumber("X", x);
			//table.putNumber("Y", y);
			//x += 0.05;
			//y += 1.0;
			

		
		}

	}
	
	
	/* This function is called periodically during test mode 
	public void testPeriodic() {
		LiveWindow.run();
	}
	*/
	
	
	
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

	/*SENSOR ACCESSOR METHODS */
	
	public double getDistance(){
		
		double x = ultraSonic.getAverageVoltage();
		double imani = 20*x*x + 2.56*x + 12.45;
        SmartDashboard.putNumber("Ahmed ultra", x);
        SmartDashboard.putNumber("Ahmed imaniUltra", imani);
    	return imani;
        
	}
	
	public void checkGyro(){
		boolean zero_yaw_pressed = driverStick.getTrigger();
        if ( zero_yaw_pressed ) {
            ahrs.zeroYaw();
        }

        /* Display 6-axis Processed Angle Data                                      */
        SmartDashboard.putBoolean(  "IMU_Connected",        ahrs.isConnected());
        SmartDashboard.putBoolean(  "IMU_IsCalibrating",    ahrs.isCalibrating());
        SmartDashboard.putNumber(   "IMU_Yaw",              ahrs.getYaw());
        SmartDashboard.putNumber(   "IMU_Pitch",            ahrs.getPitch());
        SmartDashboard.putNumber(   "IMU_Roll",             ahrs.getRoll());
        
        
        /*
        /* Display tilt-corrected, Magnetometer-based heading (requires             
        /* magnetometer calibration to be useful)                                   
        
        SmartDashboard.putNumber(   "IMU_CompassHeading",   ahrs.getCompassHeading());
        
        /* Display 9-axis Heading (requires magnetometer calibration to be useful)  
        SmartDashboard.putNumber(   "IMU_FusedHeading",     ahrs.getFusedHeading());

        /* These functions are compatible w/the WPI Gyro Class, providing a simple  
        /* path for upgrading from the Kit-of-Parts gyro to the navx-MXP            
        
        SmartDashboard.putNumber(   "IMU_TotalYaw",         ahrs.getAngle());
        SmartDashboard.putNumber(   "IMU_YawRateDPS",       ahrs.getRate());

        /* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) 
        
        SmartDashboard.putNumber(   "IMU_Accel_X",          ahrs.getWorldLinearAccelX());
        SmartDashboard.putNumber(   "IMU_Accel_Y",          ahrs.getWorldLinearAccelY());
        SmartDashboard.putBoolean(  "IMU_IsMoving",         ahrs.isMoving());
        SmartDashboard.putBoolean(  "IMU_IsRotating",       ahrs.isRotating());

        /* Display estimates of velocity/displacement.  Note that these values are  
        /* not expected to be accurate enough for estimating robot position on a    
        /* FIRST FRC Robotics Field, due to accelerometer noise and the compounding 
        /* of these errors due to single (velocity) integration and especially     
        /* double (displacement) integration.                                       
        
        SmartDashboard.putNumber(   "Velocity_X",           ahrs.getVelocityX());
        SmartDashboard.putNumber(   "Velocity_Y",           ahrs.getVelocityY());
        SmartDashboard.putNumber(   "Displacement_X",       ahrs.getDisplacementX());
        SmartDashboard.putNumber(   "Displacement_Y",       ahrs.getDisplacementY());
        
        /* Display Raw Gyro/Accelerometer/Magnetometer Values                       
        /* NOTE:  These values are not normally necessary, but are made available   
        /* for advanced users.  Before using this data, please consider whether     
        /* the processed data (see above) will suit your needs.                     
        
        SmartDashboard.putNumber(   "RawGyro_X",            ahrs.getRawGyroX());
        SmartDashboard.putNumber(   "RawGyro_Y",            ahrs.getRawGyroY());
        SmartDashboard.putNumber(   "RawGyro_Z",            ahrs.getRawGyroZ());
        SmartDashboard.putNumber(   "RawAccel_X",           ahrs.getRawAccelX());
        SmartDashboard.putNumber(   "RawAccel_Y",           ahrs.getRawAccelY());
        SmartDashboard.putNumber(   "RawAccel_Z",           ahrs.getRawAccelZ());
        SmartDashboard.putNumber(   "RawMag_X",             ahrs.getRawMagX());
        SmartDashboard.putNumber(   "RawMag_Y",             ahrs.getRawMagY());
        SmartDashboard.putNumber(   "RawMag_Z",             ahrs.getRawMagZ());
        SmartDashboard.putNumber(   "IMU_Temp_C",           ahrs.getTempC());
        
        /* Omnimount Yaw Axis Information                                           
        /* For more info, see http://navx-mxp.kauailabs.com/installation/omnimount  
        AHRS.BoardYawAxis yaw_axis = ahrs.getBoardYawAxis();
        SmartDashboard.putString(   "YawAxisDirection",     yaw_axis.up ? "Up" : "Down" );
        SmartDashboard.putNumber(   "YawAxis",              yaw_axis.board_axis.getValue() );
        
        /* Sensor Board Information                                                 
        SmartDashboard.putString(   "FirmwareVersion",      ahrs.getFirmwareVersion());
        
        /* Quaternion Data                                                          
        /* Quaternions are fascinating, and are the most compact representation of  
        /* orientation data.  All of the Yaw, Pitch and Roll Values can be derived  
        /* from the Quaternions.  If interested in motion processing, knowledge of  
        /* Quaternions is highly recommended.                                       
        SmartDashboard.putNumber(   "QuaternionW",          ahrs.getQuaternionW());
        SmartDashboard.putNumber(   "QuaternionX",          ahrs.getQuaternionX());
        SmartDashboard.putNumber(   "QuaternionY",          ahrs.getQuaternionY());
        SmartDashboard.putNumber(   "QuaternionZ",          ahrs.getQuaternionZ());
        
        // Connectivity Debugging Support                                           
        SmartDashboard.putNumber(   "IMU_Byte_Count",       ahrs.getByteCount());
        SmartDashboard.putNumber(   "IMU_Update_Count",     ahrs.getUpdateCount());
  
	*/
		
	}
	

	/* BASE AUTO FUNCTIONS */
	public void goForwardAtSpeed(double speed) {
		robotDrive.mecanumDrive_Cartesian(0, 0, speed, 0);
	}
	
	public void goBackwardAtSpeed (double speed){
		robotDrive.mecanumDrive_Cartesian(0,0,-speed,0);
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
	
	//this method finds the coordinates of the peg -Imani L
		public void findPeg() {
			pegX = (target1x + target2x ) /2;
			pegY = (target1y + target2y ) /2;
			finalPegx = pegX;
			finalPegy = pegY;
		}
		
		//this method places a gear on a peg -Shivanie H
		public void placePeg() {
		
		
		}		

		//Simple Go forward AUTO strategy -Imani L, Ryan T, and Ahmed A
		public void runAutoStrategy_GoForwardOnly(Timer timerAuto) {
			double timerA = timerAuto.get();
			if(timerA < 2) {
				goForwardAtSpeed(0.3);
			}
			else{
				stopDrive();
			}
		}
		
		//Place 1 gear on the LEFT peg Auto Strategy -Shivanie H & Jayda W
		public void runAutoStrategy_PlaceGearLeftPeg(Timer timerAuto) {
			double timerA = timerAuto.get();
			
			if(timerA < 2.0) {
				goForwardAtSpeed(0.3);
			}else if(timerA < 3.0){
				turnRightAtSpeed(0.3);
			} else if (timerA < 4.0){ 
				goForwardAtSpeed(0.3);
			} else if (timerA < 5.0){
				placePeg();
			} else if (timerA < 6.0) {
			    goBackwardAtSpeed(0.3);
			} else if(timerA < 7.0) {
			    turnLeftAtSpeed(0.3);
			} else if (timerA < 8.0){
				goForwardAtSpeed(0.3);
			} else{
				stopDrive();
			}
			
			
		}		
	
	/*
	//method to be used aim in autonomous mode -Keon
	public void aimRobot() { 
		
		while(ultraSonic.getRangeInches() > distanceFromWall) { //to loop until at acceptable distance (Within Range)
			int finalPegX = pegX; // Updates the x value of the "Peg"
			//int finalPegY = pegY;
			int ultraSonic = distanceFromWall; //Updates distance/Value
			if(finalPegX < (targetX - range)) { 
				strafeLeftAtSpeed(0.3);
			}
			else if (finalPegX > (targetX + range) {
				strafeRightAtSpeed(0.3);
			}else{
				goForwardAtSpeed(0.3);
			}
		}
		
	*/	



}
	
