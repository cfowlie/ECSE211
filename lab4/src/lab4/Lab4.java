package lab4;

import odometer.Display;
import odometer.Odometer;
import odometer.OdometerExceptions;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class Lab4 {

	// Motor Objects
	public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));

	//Ultrasonic Sensor
	private static final Port usPort = LocalEV3.get().getPort("S2");

	// Light Sensor
	private static final Port lightPort = LocalEV3.get().getPort("S1");

	private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	public static final double WHEEL_RAD = 2.095;
	public static final double TRACK = 8.5;

	public static void main(String[] args) throws OdometerExceptions {

		// Odo
		Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
		Display odometryDisplay = new Display(lcd); // No need to change

		// Ultrasonic Sensor
		SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
		SampleProvider usDistance = usSensor.getMode("Distance"); // usDistance provides samples from
		
		// Localizers
		final UltrasonicLocalizer ultrasonicLocalizer = new UltrasonicLocalizer();
		final LightLocalizer lightLocalizer = new LightLocalizer();

		// clear the display
		lcd.clear();

		// ask the user whether the motors should drive in a square or float
		lcd.drawString("< Left | Right >", 0, 0);
		lcd.drawString("       |        ", 0, 1);
		lcd.drawString(" Ultra | Light  ", 0, 2);
		lcd.drawString("sonic  |        ", 0, 3);
		int buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)

		if (buttonChoice == Button.ID_LEFT) {
			// clear the display
			lcd.clear();

			// ask the user whether the motors should drive in a square or float
			lcd.drawString("< Left | Right >", 0, 0);
			lcd.drawString("       |        ", 0, 1);
			lcd.drawString(" fall  | rise   ", 0, 2);
			lcd.drawString(" edge  | edge   ", 0, 3);
			buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)

			if (buttonChoice == Button.ID_LEFT) {
				// Display changes in position as wheels are (manually) moved
				Thread odoThread = new Thread(odometer);
				odoThread.start();
				Thread odoDisplayThread = new Thread(odometryDisplay);
				odoDisplayThread.start();
				
				// Falling edge
				(new Thread() {
					public void run() {
						ultrasonicLocalizer.fallingEdge();
					}
				}).start();
			}
			else if (buttonChoice == Button.ID_RIGHT) {
				// Display changes in position as wheels are (manually) moved
				Thread odoThread = new Thread(odometer);
				odoThread.start();
				Thread odoDisplayThread = new Thread(odometryDisplay);
				odoDisplayThread.start();
				
				// Rising edge
				(new Thread() {
					public void run() {
						ultrasonicLocalizer.risingEdge();
					}
				}).start();
			}
			
			

		} else if (buttonChoice == Button.ID_RIGHT) {
			(new Thread() {
				public void run() {
					// Light Localizer
//					lightLocalizer.findOrigin(leftMotor, rightMotor, leftRadius, rightRadius, track);
				}
			}).start();

			// Display changes in position as wheels are (manually) moved
			Thread odoThread = new Thread(odometer);
			odoThread.start();
			Thread odoDisplayThread = new Thread(odometryDisplay);
			odoDisplayThread.start();

		}
	}
}
