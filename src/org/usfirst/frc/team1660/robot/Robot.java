package org.usfirst.frc.team1660.robot;

/* ----------	IMPORTED LIBRARIES & CLASSES	--------------------------------------------------------------------------*/
import org.usfirst.frc.team1660.robot.HKdrive;
import org.usfirst.frc.team1660.robot.HKcam;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import com.ctre.CANTalon;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import org.opencv.core.Rect;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.SensorBase;
import edu.wpi.first.wpilibj.Relay;


public class Robot extends SampleRobot {

	/* ----------	Robot VARIABLES/FIELDS	--------------------------------------------------------------------------*/

	AHRS ahrs;
	HKdrive robotDrive;
	HKcam hkcam;
	boolean gyroFlag = false;

	CANTalon frontLeft = new CANTalon(1);
	CANTalon rearLeft = new CANTalon(2);
	CANTalon frontRight = new CANTalon(3);
	CANTalon rearRight = new CANTalon(4);
	CANTalon climber = new CANTalon(5);
	CANTalon miniGearFRight = new CANTalon(6);
	CANTalon miniGearFLeft = new CANTalon(7);

	Relay compressorRelay = new Relay(0);
	Relay hockeyRelay = new Relay(1);
	Relay hovaRelay = new Relay(2);

	AnalogInput ultraSonicLong = new AnalogInput(0);
	DigitalOutput trigger = new DigitalOutput(8);
	DigitalInput echo = new DigitalInput (9);
	Ultrasonic ultraSonicShort = new Ultrasonic(trigger,echo);
	DigitalInput gearDetector = new DigitalInput(0);
	DigitalInput pressureSwitch = new DigitalInput(1);


	/* SmartDashboard objects  */
	SendableChooser startingPosition;
	SendableChooser strategy;


	/* Xbox controllers Setup -Jamesey	*/
	final int A_BUTTON = 1;
	final int B_BUTTON = 2;
	final int X_BUTTON = 3;
	final int Y_BUTTON = 4;
	final int LB_BUTTON = 5;
	final int RB_BUTTON = 6;
	final int BACK_BUTTON = 7;
	final int START_BUTTON = 8;
	final int LEFT_JOY_BUTTON = 9;
	final int RIGHT_JOY_BUTTON = 10;
	final int LEFT_X_AXIS = 0;
	final int LEFT_Y_AXIS = 1;
	final int LT_AXIS = 2;
	final int RT_AXIS = 3;
	final int RIGHT_X_AXIS = 4;
	final int RIGHT_Y_AXIS = 5;
	final int POV_UP = 0;
	final int POV_LEFT = 270;
	final int POV_DOWN = 180;
	final int POV_RIGHT = 90;

	Joystick driverStick = new Joystick(0);
	Joystick manipStick = new Joystick(1);
	final int FORWARDBACKWARD_AXIS = LEFT_Y_AXIS; //Left joystick up and down
	final int TURNSIDEWAYS_AXIS = RIGHT_X_AXIS; //Right joystick side to side
	final int STRAFE_AXIS = LEFT_X_AXIS; //Left joystick side to side


	/*Lift peg coordinates that the robot sees	*/
	Rect r0 = new Rect();
	Rect r1 = new Rect();
	int target1x;
	int target1y;
	int target2x;
	int target2y;
	int pegX;
	int pegY;
	double distanceFromWall = -2.0;


	/* ----------	REQUIRED METHODS	--------------------------------------------------------------------------*/
	/* Robot(), robotInit(), autonomous(), and operatorControl()	*/

	public Robot() {

		robotDrive = new HKdrive(frontLeft, rearLeft, frontRight, rearRight);
		robotDrive.setExpiration(0.1);

		try {
			ahrs = new AHRS(SPI.Port.kMXP); //navX-MXP initialized with (SPI, I2C, TTL UART) and USB //http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
		} catch (RuntimeException ex ) {
			DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
		}

	}

	/* This function is run when the robot is first started up and should be used for any initialization code. */
	public void robotInit() {
		
		hkcam = new HKcam();
		hkcam.camInit();

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
		robotDrive.setSafetyEnabled(false);
		robotDrive.drive(-0.5, 0.0);

		Timer.delay(2.0);
		robotDrive.drive(0.0, 0.0);
		Timer timerAuto = new Timer();
		timerAuto.start(); 
		//	 int currentStrategy = (int) strategy.getSelected(); 
		while(isAutonomous() && isEnabled()){ 

			double timerA = timerAuto.get();
			SmartDashboard.putNumber("match time",timerA);
			//	   if(currentStrategy == 1) {
			runAutoStrategy_GoForwardOnly(timerAuto); 
			// runAutoStrategy_PlaceGearLeftPeg(timerAuto);
			//     }  

		}


	}


	public void operatorControl() {
		//System.out.println("operatorControl");
		robotDrive.setSafetyEnabled(true);

		while (isOperatorControl() && isEnabled()) {

			/*Driving commands		*/
			checkDriving();
			getGyro();

			/*Gear Collection commands			*/
			checkMiniGears();
			isGear();
			checkHockey();
			checkHova();
			checkCompressor();
			//checkCompressorSwitch();

			/* Gear Placing commands		*/
			//getPegCoordinates();
			getDistanceFar();
			getDistanceClose();
			//placePeg();
			
			/* Climbing commands	*/
			checkClimbRope();
			
			Timer.delay(0.005); // wait 5ms to avoid hogging CPU cycles

		}
	}


	/* ----------	OPERATOR CONTROL METHODS	--------------------------------------------------------------------------*/

	/* Mecanum Driving with XBox 360 Joysticks -Malachi P	*/
	public void checkDriving()
	{

		double threshold = 0.11;
		double strafe = squareInput(driverStick.getRawAxis(STRAFE_AXIS)) ; // right and left on the left thumb stick?
		double moveValue = squareInput(driverStick.getRawAxis(FORWARDBACKWARD_AXIS));// up and down on left thumb stick?
		double rotateValue = squareInput(driverStick.getRawAxis(TURNSIDEWAYS_AXIS));// right and left on right thumb stick
		double angle = ahrs.getAngle();

		//Kill Ghost motors -Matthew M
		if(moveValue > threshold*-1 && moveValue < threshold) {
			moveValue = 0;
		}
		if(rotateValue > threshold*-1 && rotateValue < threshold) {
			rotateValue = 0;
		}
		if(strafe > threshold*-1 && strafe < threshold) {
			strafe = 0;
		}

		//MECANUM -Malachi P
		SmartDashboard.putNumber("move",	moveValue);
		SmartDashboard.putNumber("rotate",	rotateValue);
		SmartDashboard.putNumber("strafe",	strafe);
		SmartDashboard.putNumber("angle",	angle);

		if(gyroFlag == true){
			robotDrive.mecanumDrive_Cartesian( strafe, -rotateValue, -moveValue, angle);

		} else {
			robotDrive.mecanumDrive_Cartesian( strafe, -rotateValue, -moveValue, 0);
		}
	}

	public double squareInput(double x) {
		if(x > 0 ) {
			return Math.pow(x, 4);
		}
		else{
			return -1*Math.pow(x, 4); 
		}
	}

	/* Joystick method to eat and spit gears on ground	*/
	public void checkMiniGears() {
		//	int thresh = .2;
		//double x = manipStick.getRawButton(A_BUTTON);

		if(manipStick.getRawButton(B_BUTTON)==true){
			takeMiniGears();
		}
		else if (manipStick.getRawButton(A_BUTTON)==true)
		{
			spitMiniGears();
		}

		else{
			stopMiniGears();
		}
	}


	/* Joystick Method to rotate the Gears/hova up from ground in positino to score	-Jamesey	*/
	public void checkHova(){
		if(manipStick.getRawButton(Y_BUTTON) == true){
			rotateUp();
		}
		if(manipStick.getRawButton(X_BUTTON) == true){
			rotateDown();
		}
	}

	/* Joystick method to grab and ungrab the gears with the hockey-stick shaped claw -imani l */
	public void checkHockey(){
		if(manipStick.getRawButton(LB_BUTTON) == true){
			holdGear();
		}
		if(manipStick.getRawButton(RB_BUTTON) == true){
			dropGear();
		}
	}



	public void checkClimbRope(){

		double thresh = 0.2;
		double upSpeed = manipStick.getRawAxis(LT_AXIS);
		double downSpeed = manipStick.getRawAxis(RT_AXIS);

		if(upSpeed > thresh){
			climbUp(upSpeed);
		} else if(downSpeed > thresh){
			climbDown(downSpeed);
		} else {
			dontClimb();
		}
	} 

	/*	method to turn compressor on or off	-Malachi P	*/
	public void checkCompressor(){

		if(manipStick.getPOV() == POV_UP){
			this.compressorOn();
			SmartDashboard.putString("Compressor: ", "ON-button");
		}
		else if(manipStick.getPOV() == POV_DOWN){
			this.compressorOff();
			SmartDashboard.putString("Compressor: ", "OFF-button");
		}
		
	}
	
	/* method to turn automatically turn on/off compressor based on pressure switch	-Jamesey & Donashia	*/
	public boolean checkCompressorSwitch(){
		boolean y = pressureSwitch.get();
		if( pressureSwitch.get() == true) {
			compressorOn();
		} else {
			compressorOff();
		}
		SmartDashboard.putString("Compressor: ", y + "-switch");
		return y;
	}

	
	/* method to turn robot to 90 degrees	-Malachi P	*/
	public void checkTurnRobot(){
		if(manipStick.getRawAxis(POV_LEFT)>0.5){
			//turnRobotGyro(90);
		}
	}

	/* Joystick Combo method to pick up a gear-X	*/
	/*public void checkGear(){
		
		if(manipStick.getRawButton(Y_BUTTON) == true){
			pickUp();
		
		}
			
			
		else if(manipStick.getRawButton(X_BUTTON) == false);
		        putDown();
		}
		
	*/
	

	
	
	/* Joystick Combo method to place a gear on a peg automatically	*/






	/* ----------	SENSOR ACCESSOR METHODS	--------------------------------------------------------------------------*/

	//method to be used aim in autonomous mode -Keon, Malachi P, Ahmed A
	public double getDistanceFar(){

		double x = ultraSonicLong.getAverageVoltage();
		double distanceInInches = 20*x*x + 2.56*x + 12.45;
		SmartDashboard.putNumber("Distance (Far)", distanceInInches);
		return distanceInInches;

	}
	public double getDistanceClose(){
		ultraSonicShort.setAutomaticMode(true);
		ultraSonicShort.setEnabled(true);
		
		double y = ultraSonicShort.getRangeInches();
		double  x = ultraSonicShort.getRangeInches();
		distanceFromWall = y;
		SmartDashboard.putNumber("Distance (Close)", y);
		return y;
	}

	// gearDetector to check if we have a gear, then used to move Hova up
	public boolean isGear(){
		boolean gotGear = gearDetector.get();
		SmartDashboard.putBoolean("gotGear?", gotGear);
		return gotGear;
	}

	/*	this method finds the coordinates of the peg -Imani L & Marlahna M	*/
	public void getPegCoordinates() {

		r0 = hkcam.getRect0();
		r1 = hkcam.getRect1();

		if(hkcam.getNumRectangles() > 1){ //make sure we see 2 targets

			target1x = r0.x + r0.width/2;
			target1y = r0.y + r0.height/2;
			target2x = r1.x + r1.width/2;
			target2y = r1.y + r1.height/2;

			pegX = (target1x + target2x ) /2;
			pegY = (target1y + target2y ) /2;
		}
	}

	/* method to get the angle of the robot from the navX	-Malachi P	*/
	public void getGyro(){

		//Zero the gyro/yaw of the navX
		if (driverStick.getRawAxis(LT_AXIS) > 0.5) {
			ahrs.zeroYaw();
		}

		/* Display 6-axis Processed Angle Data                                      */
		SmartDashboard.putBoolean(  "IMU_Connected",        ahrs.isConnected());
		SmartDashboard.putBoolean(  "IMU_IsCalibrating",    ahrs.isCalibrating());
		SmartDashboard.putNumber(   "IMU_Yaw",              ahrs.getYaw());
		SmartDashboard.putNumber(   "IMU_Pitch",            ahrs.getPitch());
		SmartDashboard.putNumber(   "IMU_Roll",             ahrs.getRoll()); 

		// Display tilt-corrected, Magnetometer-based heading (requires magnetometer calibration to be useful)                                   
		//SmartDashboard.putNumber(   "IMU_CompassHeading",   ahrs.getCompassHeading());

		// Display 9-axis Heading (requires magnetometer calibration to be useful)  
		//SmartDashboard.putNumber(   "IMU_FusedHeading",     ahrs.getFusedHeading());

		// These functions are compatible w/the WPI Gyro Class, providing a simple path for upgrading from the Kit-of-Parts gyro to the navx-MXP            
		SmartDashboard.putNumber(   "IMU_TotalYaw",         ahrs.getAngle());
		SmartDashboard.putNumber(   "IMU_YawRateDPS",       ahrs.getRate());

		// Display Processed Acceleration Data (Linear Acceleration, Motion Detect)        
		//SmartDashboard.putNumber(   "IMU_Accel_X",          ahrs.getWorldLinearAccelX());
		//SmartDashboard.putNumber(   "IMU_Accel_Y",          ahrs.getWorldLinearAccelY());
		//SmartDashboard.putBoolean(  "IMU_IsMoving",         ahrs.isMoving());
		//SmartDashboard.putBoolean(  "IMU_IsRotating",       ahrs.isRotating());

		// Omnimount Yaw Axis Information                                           
		// For more info, see http://navx-mxp.kauailabs.com/installation/omnimount  
		//AHRS.BoardYawAxis yaw_axis = ahrs.getBoardYawAxis();
		//SmartDashboard.putString(   "YawAxisDirection",     yaw_axis.up ? "Up" : "Down" );
		//SmartDashboard.putNumber(   "YawAxis",              yaw_axis.board_axis.getValue() );

		// Sensor Board Information                                                 
		SmartDashboard.putString(   "NavX FirmwareVersion",      ahrs.getFirmwareVersion());

	}


	/* method to get the value from a Pneumatic pressure switch	*/






	/* ----------	BASIC ROBOT FUNCTIONALITY METHODS	--------------------------------------------------------------------------*/

	/* basic "hova" rotation methods	*/
	public void rotateUp(){
		hovaRelay.set(Relay.Value.kForward); 	
	}
	public void rotateDown(){
		hovaRelay.set(Relay.Value.kReverse);	
	}

	/* basic gear grabbing methods	-Jamesey	*/
	public void holdGear(){

		this.hockeyRelay.set(Relay.Value.kForward);
	}
	public void dropGear(){
		this.hockeyRelay.set(Relay.Value.kReverse);	 
	}

	/* basic compressor functionality methods	*/
	public void compressorOn(){
		this.compressorRelay.set(Relay.Value.kForward);
		SmartDashboard.putString("compressorStatus", "is on");
	}
	public void compressorOff(){
		this.compressorRelay.set(Relay.Value.kOff);
		SmartDashboard.putString("compressorStatus", "is off");
	}

	/* basic climb method	*/
	public void climbUp(double x){
		this.climber.set(x);
	}
	public void climbDown(double x){
		this.climber.set(-x);
	}
	public void dontClimb(){
		this.climber.set(0.0);
	}


	/* basic method to take and spit gears on ground	*/
	public void takeMiniGears(){
		miniGearFRight.set(1.0);
		miniGearFLeft.set(-1.0);
	}
	public void spitMiniGears() {
		miniGearFRight.set(-1.0);
		miniGearFLeft.set(1.0);
	}
	public void stopMiniGears(){	
		miniGearFRight.set(0.0);
		miniGearFLeft.set(0.0);
	}

	/* basic method to turn gyro-driving on and off	-Malachi P	*/
	public void turnGyroOn(){
		gyroFlag = true;
	}
	public void turnGyroOff(){
		gyroFlag = false;
	}



	/* ----------	COMBO ROBOT FUNCTIONS	--------------------------------------------------------------------------*/

	/* Combo method to Pick up a Gear from the Ground -Donashia and Jamesey	*/
	Timer comboPickUpTimer = new Timer();
	Timer comboPickupTimer2 = new Timer();
	
	public void checkComboPickUpButton(){
		if(driverStick.getRawButton(Y_BUTTON) == true){
			comboPickUpTimer.reset();
		}
	}
	
	public void checkComboPickUpTimer() {
		if (comboPickUpTimer.get() != 0){
			if(comboPickUpTimer.get() < 5 || !isGear()){
				takeMiniGears();
				if(comboPickupTimer2.get() == 0){
					comboPickupTimer2.start();
				}
			}
			if(comboPickUpTimer.get() > 5.0){
				comboPickUpTimer.stop();
				comboPickUpTimer.reset();
			}
		}
		if(comboPickupTimer2.get() < 2){
			holdGear();
		}
		if(comboPickupTimer2.get() > 2 && comboPickupTimer2.get() < 6) {
			rotateUp();
		}
		if(comboPickupTimer2.get() > 6){
			comboPickupTimer2.reset();
			comboPickupTimer2.stop();
		}
	}

	
	/* Combo Method to aim and move robot towards peg	*/
	public void comboAimRobot() { 

		double targetDistanceFromWall = 14.0;
		getDistanceClose();
		int targetX = 640/2;
		int targetY = 480/2;
		int range = 10;

		while(distanceFromWall > targetDistanceFromWall) { //to loop until at acceptable distance (Within Range)
			getPegCoordinates(); 						// Updates the x & y values of the "Peg"
			getDistanceClose();  			//Updates distance/Value

			if(pegX < (targetX - range)) { 
				strafeLeftAtSpeed(0.3);
			}
			else if (pegX > (targetX + range)) {
				strafeRightAtSpeed(0.3);
			}else{
				goForwardAtSpeed(0.3);
			}
		}
	}



	/* this method places a gear on a peg -Shivanie H	*/
	public void comboPlacePeg() {





	}		

	
	

	/* ------------------------------------------------------------------------------------*/
	/* BASIC DRIVETRAIN FUNCTIONS */

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

	//method to turn to a specific field-orientation -Malachi & Ahmed
	public void turnRobotAngle(int angle){
		//robotDrive.mecanumDrive_Cartesian( strafe, -rotateValue, -moveValue, angle);
		
	}
	
	
	
	
	/* ------------------------------------------------------------------------------------*/
	/* COMBO AUTONOMOUS METHODS */

	public void strategyDestinationPin (Timer timerAuto){


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
			//placePeg();
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



}

