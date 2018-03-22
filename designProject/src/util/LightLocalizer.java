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

public class LightLocalizer {

	EV3LargeRegulatedMotor leftMotor;
	EV3LargeRegulatedMotor rightMotor;
	EV3LargeRegulatedMotor leftUpMotor;
	EV3LargeRegulatedMotor rightUpMotor;
	EV3ColorSensor lightRSensor;
	EV3ColorSensor lightLSensor;
	
	DriveManager driveManager = DriveManager.getInstance();
	SensorManager sensorManager = SensorManager.getInstance();
	

	// travel the euclidian distance of two timea lthe light radius
	
	
	public LightLocalizer() throws OdometerExceptions {
		
		// Motors
		this.leftMotor = driveManager.getLeftMotor();
		this.rightMotor = driveManager.getRightMotor();
		this.leftUpMotor = driveManager.getLeftUpMotor();
		this.rightUpMotor = driveManager.getRightUpMotor();
		
		// Sensors
		this.lightRSensor = sensorManager.getLightSensorR(); 
		this.lightLSensor = sensorManager.getLightSensorL();
	}
	
	/*
	 * Light localizer find origin method
	 * 
	 */
	public void findOrigin()
			throws InterruptedException, OdometerExceptions {

		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.stop();
			motor.setAcceleration(3000);
		}
		
		// keeps the upper motors upsraight without them moving
		leftUpMotor.stop();
		rightUpMotor.stop();

		// wait two seconds to know its the light starting
		Thread.sleep(500);

		//performs the line waiting action to put the robot in a perfect perpendicular line with the black lines
		lineLocWait();
				
		// set the y coordinate to y=light radius
		sensorManager.getOdometer().setY(DriveManager.LIGHT_RADIUS);
		
		driveManager.forwardBy(-DriveManager.LIGHT_RADIUS);
		
		driveManager.setRotSpd();

		// turn 90 degrees to go towards the x=0 line
		driveManager.turnBy(90);
	
		//performs the line waiting action to put the robot in a perfect perpendicular line with the black lines
		lineLocWait();
	
		// set the x coordinate to x= light radius
		sensorManager.getOdometer().setX(DriveManager.LIGHT_RADIUS);
		
		driveManager.forwardBy(-DriveManager.LIGHT_RADIUS);
		
		driveManager.setRotSpd();

		// turn back towards the (0,0) coordinate
		driveManager.turnBy(-90);		
		
		sensorManager.getOdometer().setXYT(0,0,0);

		// done
		driveManager.stopAll();
	}
	
	private void lineLocWait() throws InterruptedException {
		driveManager.setDriveSpd();

		// wait until black line hits one of the two light sensors
		while (sensorManager.getLine() == 0) {
			leftMotor.forward();
			rightMotor.forward();
		}
		
		//if the right light sensor hit first, stop right motor and keep left running until left light hits line
		if(sensorManager.getLine()==2) {
			rightMotor.stop(true);
			driveManager.setRotSpd();
			while (sensorManager.getLine() != 3) {
				leftMotor.forward();
			}
			leftMotor.rotate(10);
			
		}
		//if the left light sensor hit first, stop left motor and keep left running until right light hits line
		else if(sensorManager.getLine()==1) {
			leftMotor.stop(true);
			driveManager.setRotSpd();
			while (sensorManager.getLine() != 3) {
				rightMotor.forward();
			}
			rightMotor.rotate(10);
		}
		
		leftMotor.stop(true);
		rightMotor.stop(false);
		
		sensorManager.getOdometer().roundToNearest90();
	
		
		Thread.sleep(2000);
	}
	
	
}
