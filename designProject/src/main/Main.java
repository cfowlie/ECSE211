package main;

import lejos.hardware.Button;
import wifi.*;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import odometer.Display;
import odometer.OdometerExceptions;

import util.*;


public class Main {

	private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	
	public static void main(String[] args) throws OdometerExceptions, InterruptedException {

		// Init shared Managers
		final DriveManager driveManager = DriveManager.getInstance();
		final SensorManager sensorManager = SensorManager.getInstance();
		
		

		// LCD
		lcd.clear();

		// ask the user whether the motors should drive in a square or float
		lcd.drawString("	Press Any Button", 0, 0);
		Button.waitForAnyPress(); // Record choice (left or right press)
		
		

		// Odo Display
		Display odometryDisplay = new Display(lcd);
		Thread odoDisplayThread = new Thread(odometryDisplay);
		odoDisplayThread.start();

		// Localizers
		final UltrasonicLocalizer ultrasonicLocalizer = new UltrasonicLocalizer();
		final LightLocalizer lightLocalizer = new LightLocalizer();
		final CourseFollowing courseFollowing = new CourseFollowing();
		final Wifi wifi = new Wifi();

		
		// Setup Drive Thread
		driveManager.setDriveThread(new DriveThread() {
			
			@Override
			public void run() throws InterruptedException, OdometerExceptions {
				// Ultrasonic localize
				wifi.transmit();
	
				
				ultrasonicLocalizer.fallingEdge();
				
				// Light localize
				lightLocalizer.findOrigin();
				
				

				
				//courseFollowing.followCourse();
				
				
												
				completion();
			}

			@Override
			public void completion() throws OdometerExceptions {				
				driveManager.stopAll();
			}
		});

		Thread.sleep(2500); // Wait to make sure all threads have initialzed before starting drive code
		
		// Start Drive Thread Async
		driveManager.start();

		Button.waitForAnyPress(); // Wait to exit program
		System.exit(0);
	}

}
