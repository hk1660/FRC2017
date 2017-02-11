package org.usfirst.frc.team1660.robot;

import org.opencv.imgproc.Imgproc;
import org.opencv.core.Rect;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import org.usfirst.frc.team1660.robot.GripPipeline;
import edu.wpi.first.wpilibj.vision.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.lang.Thread;

public class HKcam {

	//FIELDS (initialized to avoid null-pointer exceptions)
	private Rect r0 = new Rect();
	private Rect r1 = new Rect();
	private int numRectangles = -1;
	Object camLock = new Object();

	// METHODS

	public void camInit() {

		/* Creates UsbCamera and MjpegServer [1] and connects them */
		UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
		
		camera.setFPS(30);
		camera.setWhiteBalanceAuto();

		VisionThread visionThread = new VisionThread(camera, new GripPipeline(), pipeline -> {
			try{
				int tempNumRectangles = pipeline.filterContoursOutput().size();
				
				Rect tempR0 = r0; //keep old value if you don't see it?
				
				System.out.println("\t rect count = " + tempNumRectangles);
				SmartDashboard.putNumber("rect count", tempNumRectangles);

				if (tempNumRectangles > 0){		//!pipeline.filterContoursOutput().isEmpty()) {

					tempR0 = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
					Rect tempR1 = r1;		//keep old value if you don't see it?
					
					System.out.println ("r0.x: " +r0.x + " r0.y: " +r0.y+ " r0.height: " + r0.height + " r0.width: "+r0.width);
					SmartDashboard.putNumber("rect0.x", r0.x);
					SmartDashboard.putNumber("rect0.y", r0.y);
					SmartDashboard.putNumber("rect0.height", r0.height);
					SmartDashboard.putNumber("rect0.width", r0.width);

					if (tempNumRectangles > 1) {
						tempR1 = Imgproc.boundingRect(pipeline.filterContoursOutput().get(1));
						System.out.println ("r1.x: " +r1.x + " r1.y: " +r1.y+ " r1.height: " + r1.height + " r1.width: "+r1.width);
						SmartDashboard.putNumber("rect1.x", r1.x);
						SmartDashboard.putNumber("rect1.y", r1.y);
						SmartDashboard.putNumber("rect1.height", r1.height);
						SmartDashboard.putNumber("rect1.width", r1.width);
					}
					

					synchronized(camLock){
						r0 = tempR0;
						r1 = tempR1;
						numRectangles = tempNumRectangles;
					}

					
					
				}
				
				try {
				    Thread.sleep(300);
				} catch(InterruptedException e){
					System.out.println("Thread sleep exception");
				}
			}
			catch(Exception e){
				System.out.println(e);
			}
			
		});

		visionThread.start();

	}

	public int getNumRectangles() {
		synchronized(camLock){
			return numRectangles;
		}
	}

	public Rect getRect0() {
		synchronized(camLock){
			return r0;
		}
	}

	public Rect getRect1() {
		synchronized(camLock){
			return r1;
		}
	}
	

	/* Method that prints out important things about camera to SmartDashboard -Marlahna M & Malachi P	*/
	public void camPrints() {

		System.out.print ("r0.x: \t" + getRect0().x);
		SmartDashboard.putNumber("rect0.x", getRect0().x);

		Rect tempR1 = getRect1();
		if(tempR1 != null){
			System.out.print("\t r1.x is null!");
			SmartDashboard.putString("rect1.x", "null");

		} else {
			System.out.print("\t r1.x: \t" + getRect1().x);
			SmartDashboard.putNumber("rect values", getRect1().x);
		}

		System.out.println("\t numRect = " + getNumRectangles());
		SmartDashboard.putNumber("rect values", getNumRectangles());

	}
	
	
	/* method that stops the vision processing	*/
	public void camKill(){

	}
	
	
}