package lab5;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import odometer.Display;
import odometer.Odometer;
import odometer.OdometerExceptions;

public class Lab5 {

	// Lab 5 Constants
	public static final int LLx = 2;
	public static final int LLy = 3;
	public static final int URx = 0;
	public static final int URy = 0;
	public static final int TB = 0;
	public static final int SC = 0;

	private static final TextLCD lcd = LocalEV3.get().getTextLCD();

	public static void main(String[] args) throws OdometerExceptions, InterruptedException {

		// Init shared Managers
		DriveManager driveManager = DriveManager.getInstance();
		SensorManager.getInstance();

		// LCD
		lcd.clear();

		// ask the user whether the motors should drive in a square or float
		lcd.drawString("	Press Any Button", 0, 0);
		int buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)

		// Odo Display
		Display odometryDisplay = new Display(lcd);
		Thread odoDisplayThread = new Thread(odometryDisplay);
		odoDisplayThread.start();

		// Localizers
		final UltrasonicLocalizer ultrasonicLocalizer = new UltrasonicLocalizer();
		final LightLocalizer lightLocalizer = new LightLocalizer();

		// Setup Drive Thread
		driveManager.setDriveThread(new DriveThread() {
			@Override
			public void run() throws InterruptedException {
				// Ultrasonic localize
				ultrasonicLocalizer.fallingEdge2();

				// Light localize
				lightLocalizer.findOrigin();

				// Move to starting location
				travelTo(LLx, LLy, false);

				// Begin Block Search
				blockSearch();

				completion();
			}

			@Override
			public void completion() {
				// Any code that should be run after the main drive method completes
				// Drive Thread is async, so must go here and not after thread.start()
			}
		});

		// Start Drive Thread Async
		driveManager.start();

		Button.waitForAnyPress(); // Wait to exit program
		System.exit(0);
	}

	public static void blockSearch() {
		// TODO: Search for color block
	}

	/**
	 * This method causes the robot to travel to the absolute field location (x,y),
	 * specified in tile points.
	 * 
	 * @param x
	 * @param y
	 */
	public static void travelTo(double x, double y, boolean avoid) {

		
	}

}
