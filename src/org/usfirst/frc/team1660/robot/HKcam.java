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
	boolean camRunning = true;

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


				/*Find the two biggest rectangles --Khalil and Marlahna	*/
				Rect maxRect1 = new Rect;
				Rect maxRect2 = new Rect;

				int tempNumRectangles = pipeline.filterContoursOutput().size();

				//check if rec
				if(  (tempNumRectangles   >= 2) {



					//storage
					maxRect1 = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
					maxRect2 = Imgproc.boundingRect(pipeline.filterContoursOutput().get(1));

					//check each element
					for(int i = 1; i < tempNumRectangles; i++) {

						//Pulls rectangle at current position in the arraylist at pos i
						Rect temp = Imgproc.boundingRect(pipeline.filterContoursOutput().get(i));


						//case1: largest
						if(temp.area() > maxRect1.area()) {

							maxRect2 = maxRect1;
							maxRect1 = temp;
						}

						//case2: 2nd largest
						else if(temp.area() > maxRect2.area()) {

							maxRect2 = temp;
						}


						//case: trash



					}	
				}

				synchronized(camLock){
					r0 = maxRect1;
					r1 = maxRect2;
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
		camRunning = false;
		
		
		
	}
	
	
}