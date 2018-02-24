package lab5;

import lab5.LightLocalizer;
import lab5.UltrasonicLocalizer;
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

	int x = 0;
	int y = 0;
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
	public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));

	// Ultrasonic Sensor
	private static final Port usPort = LocalEV3.get().getPort("S2");

	// Light Sensor
	private static final Port lightPort = LocalEV3.get().getPort("S1");

	private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	public static final double WHEEL_RAD = 2.095;
	public static final double TRACK = 8.5;

	public static void main(String[] args) throws OdometerExceptions, InterruptedException {

		// TODO: LCD Display here
		
		// Odometer
		Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
		Display odometryDisplay = new Display(lcd);
		Thread odoThread = new Thread(odometer);
		odoThread.start();
		Thread odoDisplayThread = new Thread(odometryDisplay);
		odoDisplayThread.start();

		// Ultrasonic Sensor
		SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
		SampleProvider usDistance = usSensor.getMode("Distance"); // usDistance provides samples from
		float[] usData = new float[usDistance.sampleSize()]; // usData is the buffer in which data are
		UltrasonicPoller usPoller = new UltrasonicPoller(usDistance, usData);
		usPoller.start();
		
		// Color Sensor
		EV3ColorSensor colorSensor = new EV3ColorSensor(LocalEV3.get().getPort("S1"));
		ColorPoller colorPoller = new ColorPoller(colorSensor);
		colorPoller.start();
	
		// Localizers
		final UltrasonicLocalizer ultrasonicLocalizer = new UltrasonicLocalizer(leftMotor, rightMotor, usPoller);
		final LightLocalizer lightLocalizer = new LightLocalizer(leftMotor, rightMotor, colorSensor);

		// Localize to correct location
		Localizer localizer = new Localizer() {
			public void localize() throws InterruptedException {
				
				// Ultrasonic localize
				ultrasonicLocalizer.fallingEdge();

				// Light localize
				lightLocalizer.findOrigin();

			}
		};

		localizer.localize();

		// Move to starting location
		navigateToStart(odometer, usPoller, colorPoller);
		
		// Begin Block Search
		blockSearch(odometer, usPoller, colorPoller);
	}

	public static void blockSearch(Odometer odometer, UltrasonicPoller usPoller, ColorPoller colorPoller) {
		// TODO: Search for color block
	}
	
	public static void navigateToStart(Odometer odometer, UltrasonicPoller usPoller, ColorPoller colorPoller) {
		// TODO: Go to search grid
	}

}
