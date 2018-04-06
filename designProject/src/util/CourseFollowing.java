package util;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import main.DriveManager;
import main.SensorManager;
import odometer.Odometer;
import odometer.OdometerExceptions;

/**
 * Course following class is meant mainly for Beta Demo as a way to hardcode the process.
 * 
 * Author: David Cast
 */

public class CourseFollowing {
	


	

	EV3LargeRegulatedMotor leftMotor;
	EV3LargeRegulatedMotor rightMotor;
	EV3LargeRegulatedMotor leftUpMotor;
	EV3LargeRegulatedMotor rightUpMotor;

	
	DriveManager driveManager = DriveManager.getInstance();
	SensorManager sensorManager = SensorManager.getInstance();
	
	

	// travel the euclidian distance of two timea lthe light radius
	
	
	public CourseFollowing() throws OdometerExceptions {
		
		// Motors
		this.leftMotor = driveManager.getLeftMotor();
		this.rightMotor = driveManager.getRightMotor();
		this.leftUpMotor = driveManager.getLeftUpMotor();
		this.rightUpMotor = driveManager.getRightUpMotor();
		
		// Sensors

	}
	
	/*
	 * Light localizer find origin method
	 * 
	 */
	public void followCourse() throws InterruptedException, OdometerExceptions {

		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.stop();
			motor.setAcceleration(3000);
		}
		
		// keeps the upper motors upsraight without them moving
		leftUpMotor.stop();
		rightUpMotor.stop();

		// wait two seconds to know its the light starting
		Thread.sleep(500);
		
		driveManager.travelTo(2, 1, true);
		driveManager.travelTo(1, 1, true);
		driveManager.travelTo(1, 2, true);
		driveManager.travelTo(2, 0, true);
		
		
		
//		if(DriveManager.GreenCorner == 2 || DriveManager.GreenCorner == 3) {
//					    	
//		    	driveManager.travelToGrid(DriveManager.TN_URx-0.5, DriveManager.TN_URy+1.5);
//		    	
//		    	
//		    	driveManager.turnTo(180);
//		    	
//		    	driveManager.lineLocWait();
//		    	
//		    	tunnelSeq();
//		    	
//		    	sensorManager.getOdometer().setX((DriveManager.TN_LLx+0.5)*DriveManager.TILE_SIZE);
//		    	sensorManager.getOdometer().setY((DriveManager.TN_LLy-1.5)*DriveManager.TILE_SIZE);
//		    	
//		    	driveManager.travelToGrid(DriveManager.BR_LLx+1, DriveManager.BR_LLy-1.5);
//		    	
//		    	driveManager.turnTo(0);
//		    	
//		    	driveManager.lineLocWait();
//		    	
//		    	bridgeSeq();
//		    	
//		    	driveManager.travelToGrid(driveManager.startCornerLoc()[0]+driveManager.startCornerLoc()[3],driveManager.startCornerLoc()[1]+driveManager.startCornerLoc()[4]);
//		 
//		}else {
//			
//		    	driveManager.travelToGrid(DriveManager.TN_LLx+0.5, DriveManager.TN_LLy-1);
//		    	
//		    	driveManager.turnTo(0);
//		    	
//		    	driveManager.lineLocWait();
//		    	
//		    	tunnelSeq();
//		    	
//		    	sensorManager.getOdometer().setX((DriveManager.TN_URx-0.5)*DriveManager.TILE_SIZE);
//		    	sensorManager.getOdometer().setY((DriveManager.TN_URy+1.5)*DriveManager.TILE_SIZE);
//		    	
//		    	driveManager.travelToGrid(DriveManager.BR_URx-1,DriveManager.BR_URy-1);
//		    	
//		    	driveManager.turnTo(180);
//		    	
//		    	driveManager.lineLocWait();
//		    	
//		    	bridgeSeq();
//		    	
//		    	driveManager.travelToGrid(driveManager.startCornerLoc()[0]+driveManager.startCornerLoc()[3],driveManager.startCornerLoc()[1]+driveManager.startCornerLoc()[4]);
//		    
//			
//		}
		// -1 in starting corner 0 for bridge BR_URy
		
	    
	    
	    
	    
		driveManager.stopAll();
	}
	
	/**
	 * Returns true if starting at the bridge first.
	 * 
	 * @return
	 */
	public boolean checkBorT() {
		double dT = Math.sqrt(Math.pow(DriveManager.TN_LLx - driveManager.startCornerLoc()[0], 2)+Math.pow(DriveManager.TN_LLy - driveManager.startCornerLoc()[1] , 2));
		double dB = Math.sqrt(Math.pow(DriveManager.BR_LLx - driveManager.startCornerLoc()[0], 2)+Math.pow(DriveManager.BR_LLy - driveManager.startCornerLoc()[1], 2));
		
		if (dB-dT>0) {
			return true;
		}else {
			return false;
		}
		
		
	}
	
	public void tunnelSeq() throws OdometerExceptions {
    	
    	driveManager.forwardBy(4.5*DriveManager.TILE_SIZE-DriveManager.LIGHT_RADIUS);
    }
    
    
    public void bridgeSeq() throws OdometerExceptions {
    	
    	driveManager.transform();
    	driveManager.forwardBy(3.5*DriveManager.TILE_SIZE-DriveManager.LIGHT_RADIUS);
    	driveManager.transform();
    	driveManager.forwardBy(1*DriveManager.TILE_SIZE);
    }
	
   
	
	
}

