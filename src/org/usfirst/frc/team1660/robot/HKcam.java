package org.usfirst.frc.team1660.robot;

import org.opencv.imgproc.Imgproc;
import org.opencv.core.Rect;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.VisionThread;
import org.usfirst.frc.team1660.robot.GripPipeline;
import edu.wpi.first.wpilibj.vision.VisionRunner;
import edu.wpi.first.wpilibj.vision.VisionThread;


public class HKcam {
	
	
	//FIELDS
	GripPipeline pipeLine;
	Rect r0;
	Rect r1;
	
	int numRectangles;


	
	
	//CONSTRUCTOR
	public HKcam() {
	
	
	}
	
	
	//METHODS
	
	
	public void camInit(){		
	
			//NetworkTable.setIPAddress("10.16.60.63");
			//table = NetworkTable.getTable("marly");
			
			/* Creates UsbCamera and MjpegServer [1] and connects them */
			UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
			
			/* Creates the CvSource and MjpegServer [2] and connects them	*/ 
			CvSource outputStream = CameraServer.getInstance().putVideo("steamVideo", 640, 480);
			
			/* Creates the CvSink and connects it to the UsbCamera */
			CvSink cvSink = CameraServer.getInstance().getVideo();
			
			//pipeline.process(camera);
			
		    VisionThread visionThread = new VisionThread(camera, new GripPipeline(), pipeline -> {
		    	
		        if (!pipeline.filterContoursOutput().isEmpty()) {
		  
		        	r0 = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
		        	SmartDashboard.putNumber("Rec0 X", r0.x );
		        	numRectangles = pipeline.filterContoursOutput().size();
		        	if(pipeline.filterContoursOutput().size() > 1){
		
			        	r1 = Imgproc.boundingRect(pipeline.filterContoursOutput().get(1));
			        	SmartDashboard.putNumber("Rec1 X", r1.x );
		        	} 
		           
		        	
		        	//System.out.println(pipeline.filterContoursOutput().get(0));
		            //SmartDashboard.putNumber("opencv",pipeline.filterContoursOutput().get(0));
		            //table = NetworkTable.getTable("GRIP/marly");
		        
		            //double[] def = new double[0];
		            //double[] areas = table.getNumberArray("width", def);
		            //System.out.println(areas[0]);
		        }
		    });
		    visionThread.start();

	}
	
	public int getNumRectangles() {
		return numRectangles;
	}
	
	public Rect getRect0(){
		return r0;
	}
		public Rect getRect1() {
			return r1;
		}
	
	}
