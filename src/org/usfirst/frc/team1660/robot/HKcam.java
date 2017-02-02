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

		// NetworkTable.setIPAddress("10.16.60.63");
		// table = NetworkTable.getTable("marly");

		/* Creates UsbCamera and MjpegServer [1] and connects them */
		UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();

		/* Creates the CvSource and MjpegServer [2] and connects them */
		CvSource outputStream = CameraServer.getInstance().putVideo("steamVideo", 640, 480);

		/* Creates the CvSink and connects it to the UsbCamera */
		CvSink cvSink = CameraServer.getInstance().getVideo();

		// pipeline.process(camera);

		VisionThread visionThread = new VisionThread(camera, new GripPipeline(), pipeline -> {
			while (true) {

				int tempNumRectangles = pipeline.filterContoursOutput().size();

				if (tempNumRectangles > 0){		//!pipeline.filterContoursOutput().isEmpty()) {

					Rect tempR0 = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
					Rect tempR1 = r1;		//keep old value if you don't see it?

					if (tempNumRectangles > 1) {
						tempR1 = Imgproc.boundingRect(pipeline.filterContoursOutput().get(1));
					}

					synchronized(camLock){
						r0 = tempR0;
						r1 = tempR1;
						numRectangles = tempNumRectangles;
					}
				}
				
				try {
				    Thread.sleep(200);
				} catch(InterruptedException e){
					System.out.println("Thread sleep exception");
				}
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