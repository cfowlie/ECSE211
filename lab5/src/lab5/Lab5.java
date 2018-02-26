package lab5;

import lab5.LightLocalizer;
import lab5.UltrasonicLocalizer;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import light.ColorPoller;
import odometer.Display;
import odometer.Odometer;
import odometer.OdometerExceptions;
import ultrasonic.UltrasonicPoller;

interface Localizer {
	void localize() throws InterruptedException;
}

public class Lab5 {

	// Lab 5 Constants
	public static final int LLx = 0;
	public static final int LLy = 0;
	public static final int URx = 0;
	public static final int URy = 0;
	public static final int TB = 0;
	public static final int SC = 0;

	private static final TextLCD lcd = LocalEV3.get().getTextLCD();


	public static void main(String[] args) throws OdometerExceptions, InterruptedException {

		// Init shared Managers
		DriveManager.getInstance();
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

		// Localize to correct location
		final Localizer localizer = new Localizer() {
						
			// TODO: Orient using SC Here, can define dynamic localization methods called by
			public void localize() throws InterruptedException {

				// Ultrasonic localize
				ultrasonicLocalizer.fallingEdge();

				// Light localize
				lightLocalizer.findOrigin();

				// If using async, use CountDownLatch here
			}
		};

		// Movement Thread
		(new Thread() {
			public void run() {
				try {

					localizer.localize();

					// Move to starting location
					navigateToStart();

					// Begin Block Search
					blockSearch();

				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}
		}).start();

		Button.waitForAnyPress(); // Wait to exit program
		System.exit(0);
	}

	public static void blockSearch() {
		// TODO: Search for color block
	}

	public static void navigateToStart() {
		// TODO: Go to search grid
	}

}
