package lab5;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;

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

	public static void main(String[] args) {
			
	}
	
	public static void localize() {
		
	}
	
}
