package lab5;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import odometer.Display;
import odometer.OdometerExceptions;

public class Lab5 {

	// Lab 5 Constants
	public static final int LLx = 1;
	public static final int LLy = 1;
	public static final int URx = 5;
	public static final int URy = 5;
	public static final int TB = 0;
	public static final int SC = 0;

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
		final BlockSearch blockSearch = new BlockSearch();

		Button.waitForAnyPress(); // Record choice (left or right press)
		
		// Setup Drive Thread
		driveManager.setDriveThread(new DriveThread() {
			@Override
			public void run() throws InterruptedException, OdometerExceptions {
				// Ultrasonic localize
				ultrasonicLocalizer.fallingEdge();

				// Light localize
				lightLocalizer.findOrigin();
								
				switch (SC) {
					case 0: 
						sensorManager.getOdometer().setXYT(0, 0, 0);
					case 1:
						sensorManager.getOdometer().setXYT(7, 0, 270);
					case 2:
						sensorManager.getOdometer().setXYT(7, 7, 180);
					case 3:
						sensorManager.getOdometer().setXYT(0, 7, 90);
					default:
						sensorManager.getOdometer().setXYT(0, 0, 0);
				}

				// Move to starting location
				driveManager.travelToGrid(LLx, LLy);
				
				// Back to 0
				driveManager.turnBy(-sensorManager.getOdometer().getXYT()[2]);
				
				// Begin Block Search
				blockSearch.search();
				
				driveManager.stopAll();
				
				// Move to ending location
				driveManager.travelToGrid(URx, URy);
												
				completion();
			}

			@Override
			public void completion() throws OdometerExceptions {				
				// Any code that should be run after the main drive method completes
				// Drive Thread is async, so must go here and not after thread.start()
				driveManager.stopAll();
				Sound.beep();
			}
		});

		Thread.sleep(2500); // Wait to make sure all threads have initialzed before starting drive code
		
		// Start Drive Thread Async
		driveManager.start();

		Button.waitForAnyPress(); // Wait to exit program
		System.exit(0);
	}

}
