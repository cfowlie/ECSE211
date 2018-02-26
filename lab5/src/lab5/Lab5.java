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

	// Motor Objects
	public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));

	// Ultrasonic Sensor
	private static final Port usPort = LocalEV3.get().getPort("S2");

	// Light Sensor
	private static final Port lightPort = LocalEV3.get().getPort("S3");
	
	// Color Sensor
	private static final Port colorPort = LocalEV3.get().getPort("S1");

	private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	public static final double WHEEL_RAD = 2.095;
	public static final double TRACK = 16.9;

	public static void main(String[] args) throws OdometerExceptions, InterruptedException {

		// LCD
		lcd.clear();

		// ask the user whether the motors should drive in a square or float
		lcd.drawString("	Press Any Button", 0, 0);
		int buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)

		// Odometer
		final Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
		Display odometryDisplay = new Display(lcd);
		Thread odoThread = new Thread(odometer);
		odoThread.start();
		Thread odoDisplayThread = new Thread(odometryDisplay);
		odoDisplayThread.start();

		// Ultrasonic Sensor
		SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
		SampleProvider usDistance = usSensor.getMode("Distance"); // usDistance provides samples from
		float[] usData = new float[usDistance.sampleSize()]; // usData is the buffer in which data are
		final UltrasonicPoller usPoller = new UltrasonicPoller(usDistance, usData);
		usPoller.start();

		// Color Sensor
		EV3ColorSensor colorSensor = new EV3ColorSensor(colorPort);
		final ColorPoller colorPoller = new ColorPoller(colorSensor);
		colorPoller.start();
		
		// Light Sensor
		EV3ColorSensor lightSensor = new EV3ColorSensor(lightPort);
		

		// Localizers
		final UltrasonicLocalizer ultrasonicLocalizer = new UltrasonicLocalizer(leftMotor, rightMotor, usPoller);
		final LightLocalizer lightLocalizer = new LightLocalizer(leftMotor, rightMotor, lightSensor);

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
					navigateToStart(odometer, usPoller, colorPoller);

					// Begin Block Search
					blockSearch(odometer, usPoller, colorPoller);

				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}
		}).start();

		Button.waitForAnyPress(); // Wait to exit program
		System.exit(0);
	}

	public static void blockSearch(Odometer odometer, UltrasonicPoller usPoller, ColorPoller colorPoller) {
		// TODO: Search for color block
	}

	public static void navigateToStart(Odometer odometer, UltrasonicPoller usPoller, ColorPoller colorPoller) {
		// TODO: Go to search grid
	}

}
