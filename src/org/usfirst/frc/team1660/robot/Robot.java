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
	boolean gyroFlag = true;	//use gyro for driving
	boolean autoDriveFlag = false;	//automatic driving
	boolean rotateFlag = false;	//helps open claw after rotating down
	Timer timerR = new Timer();


	double rotateToAngleRate;
	double zeroedYawPoint = 0.0;

	CANTalon frontLeft = new CANTalon(1);
	CANTalon rearLeft = new CANTalon(2);
	CANTalon frontRight = new CANTalon(3);
	CANTalon rearRight = new CANTalon(4);
	CANTalon climber = new CANTalon(5);
	CANTalon miniGearFRight = new CANTalon(6);
	CANTalon miniGearFLeft = new CANTalon(7);
double lastUsedAngle;
	Relay compressorRelay = new Relay(0);
	Relay hockeyRelay = new Relay(1);
	Relay hovaRelay = new Relay(2);

	AnalogInput ultraSonicLong = new AnalogInput(0);
	DigitalInput trigger = new DigitalInput(8);
	DigitalOutput echo = new DigitalOutput (9);
	Ultrasonic ultraSonicShort = new Ultrasonic(echo, trigger);
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

		/*
		startingPosition = new SendableChooser();
		startingPosition.addDefault("Left", new Integer(1));
		startingPosition.addObject("Middle", new Integer(2));
		startingPosition.addObject("Right", new Integer(3));
		SmartDashboard.putData("startingPosition", startingPosition);
		 */

		strategy = new SendableChooser();
		strategy.addDefault("Move forward only", new Integer(1));
		strategy.addObject("Place Gear Left-Peg", new Integer(2));
		strategy.addObject("No-Cam Front-Peg", new Integer(3));
		strategy.addObject("No-Cam Left-Peg", new Integer(4));
		SmartDashboard.putData("strategy selector", strategy);

		//Start match with zeroed gyro, and grabbing gear down
		zeroedYawPoint = ahrs.getAngle();
		rotateDown();
		holdGear();

	}


	/* This function is called periodically during autonomous */
	public void autonomous() {
		

		//This should be tested
		
		changeDrivingToVoltage();

		robotDrive.setSafetyEnabled(false);
		Timer timerAuto = new Timer();
		timerAuto.start(); 
		int currentStrategy = (int) strategy.getSelected(); 
		zeroedYawPoint = ahrs.getAngle();
		while(isAutonomous() && isEnabled()){ 

			double timerA = timerAuto.get();
			SmartDashboard.putNumber("AutoTimer",timerA);
			runAutoStrategy_noCamFrontPeg(timerAuto);
		//	runAutoStrategy_noCamSidePegLeft(timerAuto);
		//	runAutoStratgy_noCamSidePegRight(timerAuto);
			
			/*
			if(currentStrategy == 1) {
				runAutoStrategy_GoForwardOnly(timerAuto);

			} else if (currentStrategy == 2) {
				runAutoStrategy_PlaceGearLeftPeg(timerAuto);

			} else if (currentStrategy == 3){
				runAutoStrategy_noCamFrontPeg(timerAuto);

			} else if (currentStrategy == 4) {
				runAutoStratgy_noCamSidePegRight(timerAuto);
			}
			*/
		}
	}


	public void operatorControl() {
		//System.out.println("operatorControl");
		this.changeDrivingToPercent();
		
		robotDrive.setSafetyEnabled(true);

		while (isOperatorControl() && isEnabled()) {

			/*Driving commands		*/
			
			checkDriving();
			//checkGyroFlag(); //always keep on!
			checkAutoTurn();
			checkResetGyro();
			printGyro();
			getCurrentAngle();
			checkCamStrafe();
			
			/*Gear Collection commands			*/
			checkMiniGears();
			isGear();
			checkHockey();
			checkHova();
			//checkCompressor();
			checkCompressorSwitch();
			//checkComboGroundLoad();


			/* Gear Placing commands		*/
			getPegX();
			getDistanceUS();
			//checkComboAimRobot();	//60 degrees?
			checkComboPlaceGear();

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

		//Kill Ghost motors & turn-off Auto methods if a joystick is pushed	-Matthew M
		if(moveValue > -threshold && moveValue < threshold) {
			moveValue = 0.0;
		}

		if(rotateValue > -threshold && rotateValue < threshold) {
			rotateValue = 0.0;
		} 

		if(strafe > -threshold && strafe < threshold) {
			strafe = 0.0;
		}

		//MECANUM -Malachi P
		if(autoDriveFlag == false ){
			robotDrive.mecanumDrive_Cartesian( strafe, -rotateValue, -moveValue, 0);

			//Prints
			SmartDashboard.putNumber("move",	moveValue);
			SmartDashboard.putNumber("rotate",	rotateValue);
			SmartDashboard.putNumber("strafe",	strafe);
			SmartDashboard.putNumber("angle",	angle);
		} 

		SmartDashboard.putBoolean("AUTODRIVING", autoDriveFlag);
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
			zeroedYawPoint = ahrs.getAngle();
			//ahrs.zeroYaw();
		}
	}

	/* joystick method to flip the gyroflag -Malachi P	*/
	public void checkCamStrafe(){
		if(driverStick.getRawButton(A_BUTTON)==true){ 
			camStrafe();
		}
	}

	
	
	/* Joystick method to eat and spit gears on ground	*/
	public void checkMiniGears() {
		double thresh = 0.2;

		if(manipStick.getRawAxis(RIGHT_Y_AXIS) > thresh){
			takeMiniGears();
		} else if (manipStick.getRawAxis(RIGHT_Y_AXIS) < -thresh){
			spitMiniGears();
		} else {
			stopMiniGears();
		}
	}

	
	
	
	
	
	/* Joystick Method to rotate the Gears/hova up from ground in positino to score	-Jamesey	*/
	public void checkHova(){
		if(manipStick.getPOV()==this.POV_UP){
			//holdGear();
			rotateUp();
		}
		if(manipStick.getPOV()==this.POV_DOWN){

			//	holdGear();
				rotateDown();
				
		}
	
	}

	/* Joystick method to grab and ungrab the gears with the hockey-stick shaped claw -imani l */
	public void checkHockey(){
		if(manipStick.getPOV() == this.POV_RIGHT){
			holdGear();
		}
		if(manipStick.getPOV() == this.POV_LEFT){
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

	/* method to turn robot to different angles automatically	-Malachi P & Jamzii	& Ahmed A	*/
	public void checkAutoTurn(){

		if(driverStick.getPOV()==this.POV_LEFT){
			autoDriveFlag = true;
			this.autoTurn(300);		//aiming at the Right Peg
			this.lastUsedAngle=300;

		}
		else if(driverStick.getPOV()==this.POV_DOWN){
			autoDriveFlag = true;
			this.autoTurn(180);	//heading back towards driverStation
			this.lastUsedAngle=180;

		}
		else if(driverStick.getPOV()==this.POV_RIGHT){
			autoDriveFlag = true;
			this.autoTurn(60);		//aiming at the Left Peg
			this.lastUsedAngle=60;

		}
		else if(driverStick.getPOV()==this.POV_UP){
			autoDriveFlag = true;
			this.autoTurn(0);	//heading away from driverStation
			this.lastUsedAngle=0;
		}
		else{
			autoDriveFlag=false;
		}
	}

	/* Joystick Combo method to pick up a gear -Donashia	*/
	public void checkComboGroundLoad(){
		if(manipStick.getRawButton(Y_BUTTON) == true){
			this.comboGroundLoad();
		}
	}

	/* Joystick combo method to aim robot driving towards peg	*/
	public void checkComboAimRobot(){
		if(driverStick.getRawButton(X_BUTTON) == true){
			autoDriveFlag = true;
			comboAimRobot(this.lastUsedAngle);
		}
		else{
			autoDriveFlag=false;
		}
	}	

	Timer x = new Timer();
	/* Joystick Combo method to place a gear on a peg automatically		-Shivanie H	*/
	public void checkComboPlaceGear(){
		if(manipStick.getRawButton(B_BUTTON) == true){
			comboPlaceGear();
			x.start();
		}
		if (x.get() > 1){
			dropGear();
			x.reset();
			x.stop();
		}
	}


	/* ----------	SENSOR ACCESSOR METHODS	--------------------------------------------------------------------------*/

	/*method to be used aim in autonomous mode -Keon, Malachi P, Ahmed A
	public double getDistanceFar(){

		double x = ultraSonicLong.getAverageVoltage();
		double distanceInInches = 20*x*x + 2.56*x + 12.45;
		SmartDashboard.putNumber("Distance (Far)", distanceInInches);
		return distanceInInches;
	}
	 */
	public double getDistanceUS(){

		ultraSonicShort.setAutomaticMode(true);
		ultraSonicShort.setEnabled(true);
		double y = ultraSonicShort.getRangeInches();
		if (y>120){
			y=-5;//this means it isnt working
		}
		SmartDashboard.putNumber("Ultra Sonic DIstance is ", y);
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

		int pegX = -1;

		int leftX = hkcam.getLeftMost();
		int rightX = hkcam.getRightMost();

		if(hkcam.getNumRectangles() > 1){ //make sure we see at least 2 targets
			pegX = (leftX + rightX) /2;
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
		SmartDashboard.putBoolean("GyroFlag", gyroFlag);
	}
	public void turnGyroOff(){
		gyroFlag = false;
		SmartDashboard.putBoolean("GyroFlag", gyroFlag);
	}

	/* method to determine the current angle/heading of the robot from 0 - 359 degrees	-Ahmed	*/
	public int getCurrentAngle(){
		int moddedAngle = Math.floorMod((int)ahrs.getAngle(), 360);
		int moddedZeroedYawPoint = Math.floorMod((int)zeroedYawPoint, 360);
		int modAngle = Math.floorMod((moddedAngle - moddedZeroedYawPoint), 360);

		SmartDashboard.putNumber("modAngle", modAngle);
		return modAngle;
	}

	
	/* method to align the robot to the peg using the camera from 3 feet away -Ahmahna	*/
	public void camStrafe(){
		
		int leftmost = hkcam.getLeftMost();		//200
		int target = 300;
		int thresh = 10;
		double speed = 0.4;
		
		//strafe to the left
		if( leftmost!=0 || leftmost!=640 || leftmost!=1){
		if (leftmost - thresh < target  ){
			robotDrive.mecanumDrive_Cartesian(-speed, 0, 0, 0);
			SmartDashboard.putString("camStrafe", "STRAFE LEFT");
		} else if (leftmost + thresh > target){
			robotDrive.mecanumDrive_Cartesian(speed, 0, 0, 0);
			SmartDashboard.putString("camStrafe", "STRAFE RIGHT");
		} else {
			this.stopDrive();
			SmartDashboard.putString("camStrafe", "STOP!");
		}
		}
		else {
			stopDrive();
		}
		
		
		
	}
	
	
	
	/* method to change the angle of the robot based off the gyro  -Ahmed A & Jamzii 
	 * @PARAM futureAngle should be an int between 0 and 359	
	 * */
	public double autoTurnSpeed (double futureAngle){
		changeDrivingToPercent();
		double actualangle = this.getCurrentAngle();
		double diff = futureAngle - actualangle;	//positive deg for right turns

		// make the diff-values range from  -179 to 179
		if (diff > 180){ 			//to handle extra large right turns
			diff = diff - 360;
		} else if (diff < -180){	//to handle extra large left turns
			diff = diff + 360;
		}
		SmartDashboard.putNumber("AutoTurn Diff", diff);

		double angle_tolerance = 5.0;
		double min_speed = 0.4;
		double desired_speed = 0.0;

		//adjust speeds to decrease as you approach the desired angle

		desired_speed = (1.0-min_speed) * (Math.abs(diff)/180) + min_speed;
		//desired_speed = Math.pow(desired_speed, 2);

		// assinging speed based on positive or negative - Kahlil & Malachi P
		if(diff > angle_tolerance ){  //right hand turn - speed
			desired_speed = -desired_speed;
		} else if(diff < -angle_tolerance){ // left hand turn +speed
			desired_speed = desired_speed;		
		}else{
			desired_speed = 0.0;
		}


		SmartDashboard.putNumber("AUtoTurn Speed", desired_speed);
		//prints
		System.out.println("ANGLE: "+ actualangle + " DIFF IS: " + diff + " DESIRED SPEED IS " + desired_speed);
		return desired_speed;
	}
	public void autoTurn(int futureAngle){

		//find correct speed to turn
		double desired_speed = autoTurnSpeed(futureAngle);
		
		double strafeSpeedLeft= driverStick.getRawAxis(LT_AXIS);
		double strafeSpeedRight = driverStick.getRawAxis(RT_AXIS);
		double strafeSpeedActual=0;
		double minMotorSpeed = .3;
		if(strafeSpeedLeft>0){
			strafeSpeedActual=((Math.pow(strafeSpeedLeft,2))/.3)+minMotorSpeed;
			strafeSpeedActual=strafeSpeedActual*-1;
		}
		else if (strafeSpeedRight>0){
			strafeSpeedActual=(Math.pow(strafeSpeedRight,2)/.3)+minMotorSpeed;
			}
		
		

		//keep turning until within the tolerance from desired angle
		robotDrive.mecanumDrive_Cartesian(strafeSpeedActual, desired_speed, 0, 0);

	}


	/* ----------	COMBO ROBOT FUNCTIONS	--------------------------------------------------------------------------*/

	/* Combo method to Pick up a Gear from the Ground -Donashia and Jamesey	*/
	Timer comboGroundLoadTimer = new Timer();
	double eatTime = 0.0;
	boolean groundFlag = false;
	boolean eatFlag = false;

	public void comboGroundLoad() {

		//start the method's timer & Make sure claw is down & open
		if(groundFlag == false){
			groundFlag = true;
			comboGroundLoadTimer.reset();
			comboGroundLoadTimer.start();

			this.rotateDown();
			this.dropGear();
		}

		//try to grab a gear for at most 5 seconds
		if(comboGroundLoadTimer.get() < 5.0 && isGear() == false){
			takeMiniGears();			
		}

		//record the time that the Gear was grabbed
		if(eatFlag == false && isGear() == true){
			eatTime = comboGroundLoadTimer.get();
			eatFlag = true;
		}

		//spin mini-gears when grabbing
		if(((comboGroundLoadTimer.get() - eatTime) > 0.5) && ((comboGroundLoadTimer.get() - eatTime) < 1.0)){
			takeMiniGears();
		}

		//grab the gear 1 second after eating it (or if time up), stop timers & reset flags
		if((comboGroundLoadTimer.get() - eatTime > 1.0) || comboGroundLoadTimer.get() > 5.0){
			holdGear();			

			comboGroundLoadTimer.reset();
			comboGroundLoadTimer.stop();
			eatTime = 0.0;
			groundFlag = false;
			eatFlag = false;
		}
	}

	/* Combo method to reposition the GearScorer after a score or drop -Ahmed A & Jamesey	*/
	Timer repoTimer = new Timer();
	boolean repoFlag = false;

	public void comboReposition(){

		//start the method's timer close the claw
		if(repoFlag == false){
			repoFlag = true;
			repoTimer.start();
			holdGear();
		} 

		//rotate hova down
		if(repoTimer.get() > 0.2){
			rotateDown();
		} 

		//open up claw to be ready for next gear
		if (repoTimer.get() > 1.5){
			dropGear();

			repoFlag = false;
			repoTimer.reset();
			repoTimer.stop();
		}
	}

	/* Combo Method to aim and move robot towards peg	*/
	public void comboAimRobot(double pegAngle) { 

		//DRIVING values
		
		double aimFwdSpeed = 0.3;
		double aimStrafeSpeed = 0.0;
		double turnSpeed = 0.0;

		//VISION values
		int leftX = hkcam.getLeftMost();
		int rightX = hkcam.getRightMost();

		int desiredCenterX = 640/2;
		int seenPeg = getPegX();
		int xDifference = desiredCenterX - seenPeg;

		int desiredWidth = 300; //this is the desired difference between leftX and rightX
		int currentWidth = rightX - leftX; // this is the current difference between the two rectangles
		int widthDifference = desiredWidth - currentWidth;

		int pixelThreshold = 10;
		double minMotorSpeed = 0.4;


		//STRAFE if not aligned to Peg
		if(xDifference > pixelThreshold){
			aimStrafeSpeed = (-1 * minMotorSpeed) + ((xDifference/320) / ( 1 - minMotorSpeed));
		} else if (xDifference < -pixelThreshold) {
			aimStrafeSpeed = minMotorSpeed + ((xDifference/desiredCenterX) / ( 1 - minMotorSpeed));
		} else {
			aimStrafeSpeed = 0.0;
		}

		//TURN if the angle is off from specific peg
		turnSpeed = autoTurnSpeed(pegAngle);			

		//MOVE fwd if not close enough to the wall yet

		/*
		if(currentWidth>pixelThreshold){ //this is ccalulating distance based on the camera
			aimFwdSpeed = minMotorSpeed + ((currentWidth/250) / ( 1 - minMotorSpeed));
		}

		 */
		
		
		double distance = getDistanceUS();
		/*
		//this is calculating distance based on the ultrasonic censor
		if(distance < 19 && distance!=-5){
			aimFwdSpeed=0;
		}
		else if(distance<43 || distance==-5){
			aimFwdSpeed=.4;
		}
		else if (distance>43 && distance<120 ){
			aimFwdSpeed=1.0;
		}
		*/
		
		
		
		//This is to aim using ultrasonic with the assistance of the camera
		if(distance==-5 && widthDifference>pixelThreshold){
			aimFwdSpeed = minMotorSpeed + ((widthDifference/250) / ( 1 - minMotorSpeed));

		}
		else if(distance < 19 && distance!=-5){
			aimFwdSpeed=0;
		}
		else if(distance<43){
			aimFwdSpeed=.4;
		}
		else if (distance>43 && distance<120 ){
			aimFwdSpeed=1.0;
		}



		//DRIVE robot towards peg
		//robotDrive.mecanumDrive_Cartesian(aimFwdSpeed, turnSpeed, aimStrafeSpeed, 0);
		robotDrive.mecanumDrive_Cartesian(aimStrafeSpeed, aimFwdSpeed, turnSpeed, 0);
		
		SmartDashboard.putNumber("aimFwdSpeed", aimFwdSpeed);
		SmartDashboard.putNumber("aimStrafeSpeed", aimStrafeSpeed);
		SmartDashboard.putNumber("aimTurnSpeed", turnSpeed);

	}

	/* this method places a gear on a peg -Shivanie H	*/
	public void comboPlaceGear(){
		rotateUp();
	}

	/* method to get the angle of the robot from the navX	-Malachi P	*/
	public void printGyro(){

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
	//this changes the driving to be based on direct voltage input
	public void changeDrivingToVoltage(){
		this.frontLeft.changeControlMode(CANTalon.TalonControlMode.Voltage);
		this.rearLeft.changeControlMode(CANTalon.TalonControlMode.Voltage);
		this.frontRight.changeControlMode(CANTalon.TalonControlMode.Voltage);
		this.rearRight.changeControlMode(CANTalon.TalonControlMode.Voltage);
	}
	//This changes the driving to be based on percentage of voltage
	public void changeDrivingToPercent(){
		this.frontLeft.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		this.rearLeft.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		this.frontRight.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		this.rearRight.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		this.robotDrive.setMaxOutput(1.0);	
	}

	public void goForwardAtSpeed(double speed) {
		
		robotDrive.mecanumDrive_Cartesian(0, 0,  speed, 0);
	}

	public void goBackwardAtSpeed (double speed){
		robotDrive.mecanumDrive_Cartesian(0,0,-speed,0);
	}

	public void stopDrive() {
		robotDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
	}

	public void strafeLeftAtSpeed(double speed) {
		robotDrive.mecanumDrive_Cartesian(speed, 0, 0, 0);
	}

	public void strafeRightAtSpeed(double speed) {
		robotDrive.mecanumDrive_Cartesian(-speed, 0, 0, 0);
	}

	public void turnLeftAtSpeed(double speed) {
		robotDrive.mecanumDrive_Cartesian(0, 0, -speed, 0);
	}

	public void turnRightAtSpeed(double speed) {
		robotDrive.mecanumDrive_Cartesian(0, 0, speed, 0);
	}

	public void turnAndStrafe(double strafeSpeed, double turnSpeed) {
		robotDrive.mecanumDrive_Cartesian(-strafeSpeed, 0, -turnSpeed, 0);
	}



	/* ------------------------------------------------------------------------------------*/
	/* COMBO AUTONOMOUS METHODS */


	//Simple Go forward AUTO strategy -Imani L, Ryan T, and Ahmed A
	public void runAutoStrategy_GoForwardOnly(Timer timerAuto) {
		double timeA = timerAuto.get();
		if(timeA < 1) {
			goForwardAtSpeed(0.4);
		}
		else{
			stopDrive();
		}
	}


	//Place 1 gear on the LEFT peg Auto Strategy -Shivanie H & Jayda W
	public void runAutoStratgy_noCamSidePegRight(Timer timerAuto) {
		double timeC = timerAuto.get();
if(timeC<.5){
	holdGear();
}
else if (timeC < 2.8) {
			this.goForwardVoltage(4.0);
			rotateUp();
		} else if (timeC < 4.0) {
			autoTurn(300);
		} else if (timeC < 7.0) {
			this.goForwardVoltage(4.0);
			//DEAD RECKONING
		} else if (timeC < 8.0){
			stopDrive();
			dropGear();
		}  else if (timeC < 10.0){
			this.goBackwardVoltage(4.0);
		} else {
			stopDrive();
		}


	}

	/*	method to be used aim in autonomous mode -Ahmed, Keon, Malachi P, */
	public void runAutoStratgy_noCamSidePegLeft(Timer timerAuto) {

		double timeC = timerAuto.get();
if(timeC<.5){
	holdGear();
}
else if (timeC < 2.8) {
			this.goForwardVoltage(4.0);
			rotateUp();
		} else if (timeC < 4.0) {
			autoTurn(60);
		} else if (timeC < 7.0) {
			this.goForwardVoltage(4.0);
			//DEAD RECKONING
		} else if (timeC < 8.0){
			stopDrive();
			dropGear();
		}  else if (timeC < 10.0){
			this.goBackwardVoltage(4.0);
		} else {
			stopDrive();
		}

	}


	/* Auto strategy for front peg if camera is not working	-Ahmed	*/
	public void runAutoStrategy_noCamFrontPeg(Timer timerAuto){

		double timeD = timerAuto.get();

		if(timeD < 5.0){
			goForwardVoltage(4.0);
			holdGear();
			rotateUp();
		} else if (timeD <6.0){
			stopVoltage();
			dropGear();
//goForwardAtSpeed(0.4);
	
		} else if (timeD < 8.0){
			this.goBackwardVoltage(4.0);;
		} else{
			stopVoltage();
			//stopDrive();
		}
	}
	
	public void goForwardVoltage(double newMax){
	
		
		this.frontLeft.changeControlMode(CANTalon.TalonControlMode.Voltage);
		this.rearLeft.changeControlMode(CANTalon.TalonControlMode.Voltage);
		this.frontRight.changeControlMode(CANTalon.TalonControlMode.Voltage);
		this.rearRight.changeControlMode(CANTalon.TalonControlMode.Voltage);
		
		frontLeft.set(newMax);
		frontRight.set(-newMax);
		rearLeft.set(newMax);
		rearRight.set(-newMax);
	
}

	
	public void goBackwardVoltage(double newMax){
		
		
		this.frontLeft.changeControlMode(CANTalon.TalonControlMode.Voltage);
		this.rearLeft.changeControlMode(CANTalon.TalonControlMode.Voltage);
		this.frontRight.changeControlMode(CANTalon.TalonControlMode.Voltage);
		this.rearRight.changeControlMode(CANTalon.TalonControlMode.Voltage);
		
		frontLeft.set(-newMax);
		frontRight.set(newMax);
		rearLeft.set(-newMax);
		rearRight.set(newMax);
	
}

	
	
	
	public void stopVoltage(){

			frontLeft.set(0.0);
			frontRight.set(0.0);
			rearLeft.set(0.0);
			rearRight.set(0.0);
		
	}




}