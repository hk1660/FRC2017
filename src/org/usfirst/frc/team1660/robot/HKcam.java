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
	private int vidWidth = 640;
	private int vidHeight = 480;
	private int leftMost = -1;
	private int rightMost = -1;
	private int numRectangles = -1;
	Object camLock = new Object();
	UsbCamera camera;

	// METHODS

	public void camInit() {

		/* Constructs UsbCamera and MjpegServer [1] and connects them */
		camera = CameraServer.getInstance().startAutomaticCapture();
		
		camera.setWhiteBalanceAuto();

		/* Creates the CvSource and MjpegServer [2] and connects them */
		CvSource outputStream = CameraServer.getInstance().putVideo("steamVideo", vidWidth, vidHeight);

		/* Creates the CvSink and connects it to the UsbCamera */
		CvSink cvSink = CameraServer.getInstance().getVideo();
		

		/* Constructs the VisionThread which loops between this method and the GripPipeline's process method	*/ 
		VisionThread visionThread = new VisionThread(camera, new GripPipeline(), pipeline -> {
	
			/*Find the two LEFT & RIGHT-most rectangles --Khalil and Marlahna & Jamesey	*/
			int leftMostX = vidWidth;
			int rightMostX = 0;
			int tempNumRectangles = pipeline.filterContoursOutput().size();

			//check if at least 2 rectangles
			if(  (tempNumRectangles  >= 2)) {
				
				//check each element
				for(int i = 0; i < tempNumRectangles; i++) {

					//Pulls rectangle at current position in the arraylist at pos i
					Rect temp = Imgproc.boundingRect(pipeline.filterContoursOutput().get(i));

					//case1: largest rect
					if(temp.x < leftMostX ) {
						leftMostX = temp.x;
					}

					//case2: 2nd largest rect
					else if(temp.x > rightMostX) {
						rightMostX = temp.x;
					}

					//case3: rect is trash
				}	
				
				synchronized(camLock){
					leftMost  = leftMostX ;
				    rightMost = rightMostX ;
					numRectangles = tempNumRectangles;
				}			
			}
			
			SmartDashboard.putNumber("NumRects", tempNumRectangles);
			SmartDashboard.putNumber("leftMost", leftMostX);
			SmartDashboard.putNumber("rightMost", rightMostX);
			System.out.println("NumRects: " + tempNumRectangles + "\tLeftX: " + leftMostX + "\tRightX: " + rightMostX);
			
			try {
				Thread.sleep(200);
			} catch(InterruptedException e){
				System.out.println("Thread sleep exception");
			}
		});
		visionThread.start();
		
	}

	public int getNumRectangles() {
		synchronized(camLock){
			return numRectangles;
		}

	}

	public int getLeftMost() {
		synchronized(camLock){
			return leftMost;
		}
	}

	public int getRightMost(){
		synchronized(camLock){
			return rightMost;
		}
	}

	/* method that stops the vision processing	*/
	public void camKill(){


	}


}