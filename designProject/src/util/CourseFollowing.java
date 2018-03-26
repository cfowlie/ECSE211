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

		sensorManager.getOdometer().setXYT(driveManager.TILE_SIZE,driveManager.TILE_SIZE,90);
		
		/**
		 * TODO: BUILD a code that sequences of the entire path that needs to be taken
		 * 
		 * 
		 * 
		 * 
		 */
	

		
		 
		
		
		
		

		driveManager.stopAll();
	}
	
	
	
	
}

