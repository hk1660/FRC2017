p;package org.usfirst.frc.team1660.robot;

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
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;

public class Robot extends SampleRobot implements PIDOutput {

	/* ----------	Robot VARIABLES/FIELDS	--------------------------------------------------------------------------*/

	AHRS ahrs;
	HKdrive robotDrive;
	HKcam hkcam;
	boolean gyroFlag = false;
	boolean ninjaFlag = false;
	PIDController ninjaController;
	double rotateToAngleRate;

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
	int white=0;

	/* SmartDashboard objects  */
	SendableChooser startingPosition;
	SendableChooser strategy;


	//	/* Xbox controllers Setup -Jamesey	*/
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
		Timer timerAuto = new Timer();
		timerAuto.start(); 
		int currentStrategy = (int) strategy.getSelected(); 

		while(isAutonomous() && isEnabled()){ 

			double timerA = timerAuto.get();
			SmartDashboard.putNumber("match time",timerA);
			if(currentStrategy == 1) {
				runAutoStrategy_GoForwardOnly(timerAuto);
			} else if (currentStrategy == 2) {
				//this.runAutoStrategy_PlaceGearLeftPeg(timerAuto);
				//this.runAutoStrategy_noCamFrontPeg(timerAuto);
				//this.runAutoStratgy_noCamSidePeg(timerAuto);


			}
		}
	}


	public void operatorControl() {
		//System.out.println("operatorControl");
		robotDrive.setSafetyEnabled(true);

		while (isOperatorControl() && isEnabled()) {

			/*Driving commands		*/
			checkDriving();
			checkGyroFlag();
			//checkTurnRobotAngle();
			checkChangeAngle();
			checkResetGyro();
			printGyro();

			/*Gear Collection commands			*/
			checkMiniGears();
			isGear();
			checkHockey();
			checkHova();
			//checkCompressor();
			checkCompressorSwitch();
			//checkComboPickUpGear();


			/* Gear Placing commands		*/
			//getPegX();
			getDistanceFar(); 
			getDistanceClose();
			//checkComboAimRobot();
			//checkComboPlaceGear()

			/* Climbing commands	*/
			checkClimbRope();

			/*Camera Testing		*/
			changeExposure();

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

		if(gyroFlag == true && ninjaFlag == false ){
			robotDrive.mecanumDrive_Cartesian( strafe, -rotateValue, -moveValue, angle);

		} else if(ninjaFlag == false) {
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

	/* method to reset the angle of the robot from the navX	-Malachi P	*/
	public void checkResetGyro(){

		//Zero the gyro/yaw of the navX
		if (driverStick.getRawButton(Y_BUTTON)) {
			ahrs.zeroYaw();
		}
	}

	/* Joystick method to eat and spit gears on ground	*/
	public void checkMiniGears() {
		double thresh = 0.2;

		if(manipStick.getRawAxis(LEFT_Y_AXIS) > thresh){
			takeMiniGears();
		} else if (manipStick.getRawAxis(LEFT_Y_AXIS) < -thresh){
			spitMiniGears();
		} else {
			stopMiniGears();
		}
	}

	/* joystick method to flip the gyroflag -Malachi P	*/
	public void checkGyroFlag(){
		if(driverStick.getRawButton(B_BUTTON)==true){ 
			this.turnGyroOn();
		}
		if(driverStick.getRawButton(A_BUTTON)==true){
			this.turnGyroOff();
		}
	}

	/* Joystick Method to rotate the Gears/hova up from ground in positino to score	-Jamesey	*/
	public void checkHova(){
		if(manipStick.getPOV()==this.POV_UP){
			rotateUp();

		}
		if(manipStick.getPOV()==this.POV_DOWN){
			rotateDown();
		}
	}

	/* Joystick method to grab and ungrab the gears with the hockey-stick shaped claw -imani l */
	public void checkHockey(){
		if(manipStick.getPOV() == this.POV_LEFT){
			holdGear();
		}
		if(manipStick.getPOV() == this.POV_RIGHT){
			dropGear();
		}
	}

	/* Joystick method to climb rope	*/
	public void checkClimbRope(){

		double thresh = 0.2;
		double upSpeed = manipStick.getRawAxis(LT_AXIS);
		//double downSpeed = manipStick.getRawAxis(RT_AXIS);	//Don't allow robot to backdrive the ratchet

		if(upSpeed > thresh){
			climbUp(upSpeed);
			//} else if(downSpeed > thresh){
			//climbDown(downSpeed);
		} else {
			dontClimb();
		}
	} 

	/*	method to turn compressor on or off	-Malachi P	*/
	public void checkCompressor(){

		if(driverStick.getRawAxis(this.RT_AXIS) > 0.5){
			this.compressorOn();
			SmartDashboard.putString("Compressor: ", "ON-button");
		}
		else if(driverStick.getRawAxis(this.LT_AXIS) > 0.5){
			this.compressorOff();
			SmartDashboard.putString("Compressor: ", "OFF-button");
		}
	}

	/* method to turn automatically turn on/off compressor based on pressure switch	-Jamesey & Donashia	*/
	public boolean checkCompressorSwitch(){
		boolean y = pressureSwitch.get();
		if( pressureSwitch.get() == false) {
			compressorOn();
		} else {
			compressorOff();
		}
		SmartDashboard.putString("Compressor: ", y + "-switch");
		return y;
	}

	/*Joystick method to manually adjust the exposure of the camera	-Ahmed A	*/
	public void changeExposure(){

		if(driverStick.getRawButton(RB_BUTTON) == true){
			white=white+1;
			if(white>90){
				white=90;
			}
			hkcam.camera.setExposureManual(white);
		}
		if(driverStick.getRawButton(LB_BUTTON) == true){
			white=white-1;
			if(white<0){
				white=0;
			}
			hkcam.camera.setExposureManual(white);

		}
		SmartDashboard.putNumber("white resolution", white);
	}

	/* method to turn robot to different angles automatically	-Malachi P */
	public void checkTurnRobotAngle(){

		//complete once turnRobotAngle() method works!

	}

	/* method to change the angle of the robot  -Jamzii */
	public boolean checkChangeAngle(){
		if(manipStick.getRawButton(X_BUTTON)==true){
			this.changeAngle(90);
		}
		return manipStick.getRawButton(X_BUTTON);
	}

	/* Joystick Combo method to pick up a gear -Donashia	*/
	public void checkComboPickUpGear(){
		if(manipStick.getRawButton(Y_BUTTON) == true){
			this.comboPickUpGear();
		}
	}

	/* Joystick combo method to aim robot driving towards peg	*/
	public void checkComboAimRobot(){
		if(manipStick.getRawButton(X_BUTTON) == true){
			comboAimRobot();
		}
	}	

	/* Joystick Combo method to place a gear on a peg automatically	*/
	public void checkComboPlaceGear(){
		if(manipStick.getRawButton(A_BUTTON) == true){
			comboPlaceGear();
		}
	}



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
		SmartDashboard.putNumber("Real Distance", y);
		return y;

	}

	// gearDetector to check if we have a gear, then used to move Hova up
	public boolean isGear(){
		boolean gotGear = gearDetector.get();
		SmartDashboard.putBoolean("gotGear?", gotGear);
		return gotGear;
	}

	/*	this method finds the coordinates of the peg -Imani L & Marlahna M	 & Jamesey */
	public int getPegX() {

		int leftX = hkcam.getLeftMost();
		int rightX = hkcam.getRightMost();

		if(hkcam.getNumRectangles() > 1){ //make sure we see at least 2 targets

			//target1x = r0.x + r0.width/2;
			//target1y = r0.y + r0.height/2;
			//target2x = r1.x + r1.width/2;
			//target2y = r1.y + r1.height/2;

			pegX = (leftX + rightX) /2;
			//pegY = (target1y + target2y ) /2;
		} else {
			pegX = -1;
		}
		
		return pegX;
	}


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


	/* method to change the angle of the robot  -Jamzii */
	public boolean changeAngle(double futureAngle){

		double actualangle = ahrs.getAngle();
		double angle_tolerance = 5.0;
		double diff = actualangle - futureAngle;

		double min_speed;
		if(diff > 0 ){
			min_speed = 0.4;
		} else{
			min_speed = -0.4;
		}

		double desired_speed = min_speed + (diff / 180) / (1-min_speed);

		System.out.println("CURRENT ANGLE: "+ actualangle + " DIFF IS: "+diff + " DESIRED SPEED IS " + desired_speed);

		if ( Math.abs(diff) > angle_tolerance) {
			robotDrive.mecanumDrive_Cartesian(0, desired_speed, 0, 0);
			return false;
		} else{
			robotDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
			return true;
		}

	}




	/* ----------	COMBO ROBOT FUNCTIONS	--------------------------------------------------------------------------*/

	/* Combo method to Pick up a Gear from the Ground -Donashia and Jamesey	*/
	Timer comboPickUpTimer = new Timer();--0s
	Timer comboPickupTimer2 = new Timer();--5s

	public void comboPickUpGear() {
		comboPickUpTimer.reset();
		if (comboPickUpTimer.get() != 0){
			if(comboPickUpTimer.get() < 5 || !isGear()){
				takeMiniGears();
				if(comboPickupTimer2.get() == 0){
					comboPickupTimer2.start();
				}
			}
			if(comboPickUpTimer.get() > 5.0){
				//comboPickUpTimer.stop();
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


	Timer rePositionGearTimer = new Timer();
	Timer rePositionGearTimer2 = new Timer();
	public void rePositionGear(){
		 rePositionGearTimer.reset();
	 if(rePositionGearTimer.get() != 0){
	  if(comboPickUpTimer.get() < 5){
		  holdGear(); 
		 }
	 }
	 if(comboPickUpTimer.get() > 5 && comboPickUpTimer.get() < 6){
		 rotateDown();
	 }
	 if(comboPickUpTimer.get() > 6 && comboPickUpTimer.get() < 8){
		 DropGear();
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
			getPegX(); 						// Updates the x & y values of the "Peg"
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
	public void comboPlaceGear(){


	}



	/* method to get the angle of the robot from the navX	-Malachi P	*/
	public void printGyro(){

		//Zero the gyro/yaw of the navX


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
		//SmartDashboard.putString(   "NavX FirmwareVersion",      ahrs.getFirmwareVersion());

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


	/*	method to turn to a specific field-orientation -Malachi & Ahmed	*/
	public void turnRobotAngleInit(){

		final double kP = 0.03;
		final double kI = 0.00;
		final double kD = 0.00;
		final double kF = 0.00;
		final double kToleranceDegrees = 2.0f;	//how on target the pidloop needs to be

		ninjaController = new PIDController(kP, kI, kD, kF, ahrs, this);
		ninjaController.setInputRange(-180.0f,  180.0f);
		ninjaController.setOutputRange(-1.0, 1.0);
		ninjaController.setAbsoluteTolerance(kToleranceDegrees);
		ninjaController.setContinuous(true);

		LiveWindow.addActuator("DriveSystem", "RotateController", ninjaController);
	}

	/*	method to turn to a specific field-orientation -Malachi & Ahmed	*/
	public void turnRobotAngle(float angle){

		turnRobotAngleInit();
		boolean rotateToAngle = false;

		if(driverStick.getPOV()==this.POV_UP){
			ninjaController.setSetpoint(0.0f);
			rotateToAngle = true;
		} else if(driverStick.getPOV()==this.POV_LEFT){
			ninjaController.setSetpoint(90.0f);
			rotateToAngle = true;
		} else if(driverStick.getPOV()==this.POV_DOWN){
			ninjaController.setSetpoint(179.9f);
			rotateToAngle = true;
		} else if(driverStick.getPOV()==this.POV_RIGHT){
			ninjaController.setSetpoint(-90.0f);
			rotateToAngle = true;
		}

		double currentRotationRate = 0;
		if(rotateToAngle == true){
			ninjaController.enable();
			currentRotationRate = rotateToAngleRate;
		}else {
			ninjaController.disable();
		}

		try {
			robotDrive.mecanumDrive_Cartesian(0,0,currentRotationRate, ahrs.getAngle());
		} catch( RuntimeException ex ) {
			DriverStation.reportError("Error communicating with drive system:  " + ex.getMessage(), true);
		}    

	}

	@Override
	/* This function is invoked periodically by the PID Controller, */
	/* based upon navX-MXP yaw angle input and PID Coefficients.    */
	public void pidWrite(double output) {
		rotateToAngleRate = output;
	}


	/* ------------------------------------------------------------------------------------*/
	/* COMBO AUTONOMOUS METHODS */

	public void strategyDestinationPin (Timer timerAuto){

	}

	//Simple Go forward AUTO strategy -Imani L, Ryan T, and Ahmed A
	public void runAutoStrategy_GoForwardOnly(Timer timerAuto) {
		double timeA = timerAuto.get();
		if(timeA < 2) {
			goForwardAtSpeed(0.3);
		}
		else{
			stopDrive();
		}
	}

	//Place 1 gear on the LEFT peg Auto Strategy -Shivanie H & Jayda W
	public void runAutoStrategy_PlaceGearLeftPeg() {
		Timer timeB = new Timer();
		
		//comboGrabGear();
		
		if(timeB.get() < 2.0){
			goForwardAtSpeed(0.5);
		} else if(timeB.get() < 5.0) {
			for(int i = 0; i < 10; i++){
				changeAngle(60);
			}
		} else if(timeB.get()> 5.0 && timeB.get() < 7.0){
			goForwardAtSpeed(0.3);
		} else if (timeB.get() > 7.0 && timeB.get() < 10.0){
			//comboPlaceGear();
		}
		stopDrive();
	}

	/*	method to be used aim in autonomous mode -Keon, Malachi P, Ahmed A	*/
	public void runAutoStratgy_noCamSideh(Timer timerAuto) {

		double timeC = timerAuto.get();

		// Numbers are open to edits
		if (timeC < 3.0) {
			holdGear();
			goForwardAtSpeed(0.3);
		} else if (timeC > 3.0 && timeC <= 4.0) {
			changeAngle(30);
		} else if (timeC > 4.0 && timeC < 15.0) {
			/// dead reckoning 
			goForwardAtSpeed(0.3);
			//120 inches

			// if ultra sonic is between 15 inches and 18 inches stop
			// other than that the values should always give you greater than 18 or -1
			if ((getDistanceClose() > 18 || getDistanceClose() == -1) && timeC < 12.0) {
				goForwardAtSpeed(.3);
			} else {
				stopDrive();
				if (timeC > 12.0 && timeC < 14.0)
					rotateUp();
				goForwardAtSpeed(.3);
				dropGear();
				// place the peg method
			}
		} else {
			stopDrive();
		}

	}


	/* Auto strategy for front peg if camera is not working	-Ahmed	*/
	public void runAutoStrategy_noCamFrontPeg(Timer timerAuto){

		double timeD = timerAuto.get();

		if (timeD <4.0){
			holdGear();
			goForwardAtSpeed(.3);
		} else if (timeD < 7.0){
			//stopDrive();
			strafeRightAtSpeed(0.3);
		} else if (timeD < 10.0){
			//stopDrive();
			turnRobotAngle(180);
		} else if (timeD < 12.0){
			rotateUp();
		} else if(timeD < 14.0){
			goForwardAtSpeed(.3);
		} else{
			stopDrive();
		}
	}





}