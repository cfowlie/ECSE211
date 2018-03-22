package main;

import lejos.hardware.ev3.LocalEV3;
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

public class SensorManager {

	// Singleton Object
	private static SensorManager sharedManager = null;

	// Ultrasonic Sensor
	private static final Port usPort = LocalEV3.get().getPort("S4");
	
	private static final Port lightRightPort = LocalEV3.get().getPort("S2");

	// Light Sensor
	private static final Port lightLeftPort = LocalEV3.get().getPort("S3");

	// Color Sensor
	private static final Port colorPort = LocalEV3.get().getPort("S1");

	// Odometer
	private Odometer odometer;

	// Color Sensor
	private EV3ColorSensor colorSensor = new EV3ColorSensor(colorPort);
	private ColorPoller colorPoller;

	// Light Sensor
	private EV3ColorSensor lightRightSensor;
	private EV3ColorSensor lightLeftSensor;

	// Ultrasonic Sensor
	private SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
	private SampleProvider usDistance = usSensor.getMode("Distance");
	
	private float[] usData = new float[usDistance.sampleSize()]; // usData is the buffer in which data are
	private UltrasonicPoller usPoller;
	

	private SensorManager() throws OdometerExceptions {
		
		// Odometer
		this.setOdometer(Odometer.getOdometer());
		Thread odoThread = new Thread(getOdometer());
		odoThread.start();

		// Ultrasonic Sensor
		this.setUsPoller(new UltrasonicPoller(usDistance, usData));
		this.getUsPoller().start();
		
		

		// Color Sensor
		setColorPoller(new ColorPoller(colorSensor));
		getColorPoller().start();
		
		setColorPoller(new ColorPoller(colorSensor));
		getColorPoller().start();

		// Light Sensor
		setLightSensorR(new EV3ColorSensor(lightRightPort));
		setLightSensorL(new EV3ColorSensor(lightLeftPort));
	}

	public static SensorManager getInstance() throws OdometerExceptions {
		if (sharedManager == null) {
			sharedManager = new SensorManager();
		}
		return sharedManager;
	}
	
	/*
	 * Returns Ultrasonic Sensors distance
	 */
	public int getDistance() {
		return this.getUsPoller().getDistance();
	}
	
	
	/*
	 * Returns int if currently over a line
	 * 0 -> No line
	 * 1 -> Left Line
	 * 2 -> Right Line
	 * 3 -> Both Lines
	 */
	public int getLine() {
		int ret = 0;
		if (this.lightLeftSensor.getColorID() > 10) ret +=1; 
		if (this.lightRightSensor.getColorID() > 10) ret +=2;
		return ret;
		
	}
	
	
	
	/*
	 * Returns true if currently over a line
	 */
	public boolean getLineR() {
		if (this.lightRightSensor.getColorID() < 10) {
			return false;
		}
		return true;
	}
	public boolean getLineL() {
		if (this.lightLeftSensor.getColorID() < 10) {
			return false;
		}
		return true;
	}
	
	/*
	 * Returns the Color Sensors current color
	 */
	public int getColor() {
		return this.colorPoller.getColorInt();
	}
	

	// MARK: Field Encapsulation
	
	/**
	 * @return the odometer
	 */
	public Odometer getOdometer() {
		return odometer;
	}

	/**
	 * @param odometer the odometer to set
	 */
	private void setOdometer(Odometer odometer) {
		this.odometer = odometer;
	}

	/**
	 * @return the colorPoller
	 */
	public ColorPoller getColorPoller() {
		return colorPoller;
	}

	/**
	 * @param colorPoller the colorPoller to set
	 */
	private void setColorPoller(ColorPoller colorPoller) {
		this.colorPoller = colorPoller;
	}

	/**
	 * @return the lightSensor
	 */
	public EV3ColorSensor getLightSensorR() {
		return lightRightSensor;
	}
	public EV3ColorSensor getLightSensorL() {
		return lightLeftSensor;
	}

	/**
	 * @param lightSensor the lightSensor to set
	 */
	private void setLightSensorR(EV3ColorSensor lightSensor) {
		this.lightRightSensor = lightSensor;
	}
	private void setLightSensorL(EV3ColorSensor lightSensor) {
		this.lightLeftSensor = lightSensor;
	}

	/**
	 * @return the usPoller
	 */
	public UltrasonicPoller getUsPoller() {
		return usPoller;
	}

	/**
	 * @param usPoller the usPoller to set
	 */
	private void setUsPoller(UltrasonicPoller usPoller) {
		this.usPoller = usPoller;
	}
	
	
	
	

}
