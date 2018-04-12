package main;

import lejos.hardware.Wifi;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import light.ColorPoller;
import light.LightPollerL;
import light.LightPollerR;
import odometer.Display;
import odometer.Odometer;
import odometer.OdometerExceptions;
import ultrasonic.UltrasonicPoller;

public class SensorManager {

	// Singleton Object
	private static SensorManager sharedManager = null;

	// Sensor Ports
	private static final Port usPort = LocalEV3.get().getPort("S4");
	private static final Port lightRightPort = LocalEV3.get().getPort("S2");
	private static final Port lightLeftPort = LocalEV3.get().getPort("S3");
	private static final Port colorPort = LocalEV3.get().getPort("S1");

	// Odometer
	private Odometer odometer;

	// Color Sensor
	private EV3ColorSensor colorSensor = new EV3ColorSensor(colorPort);
	private ColorPoller colorPoller;

	// Light Sensor
	private EV3ColorSensor lightRightSensor = new EV3ColorSensor(lightRightPort);
	public LightPollerR lightRightPoller;
	
	private EV3ColorSensor lightLeftSensor = new EV3ColorSensor(lightLeftPort);
	public LightPollerL lightLeftPoller;

	// Ultrasonic Sensor
	private SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
	private SampleProvider usDistance = usSensor.getMode("Distance");

	private float[] usData = new float[usDistance.sampleSize()]; // usData is the buffer in which data are
	private UltrasonicPoller usPoller;

	/*
	 * Init method
	 * sets up instances of sensors on threads and starts
	 */
	private SensorManager() throws OdometerExceptions {

		// Odometer
		this.setOdometer(Odometer.getOdometer());
		Thread odoThread = new Thread(getOdometer());
		odoThread.start();

		// Ultrasonic Sensor
		this.setUsPoller(new UltrasonicPoller(usDistance, usData));
		this.getUsPoller().start();

		// Color Sensors
		setColorPoller(new ColorPoller(colorSensor));
		getColorPoller().start();

		// Light Sensors
		setLightRPoller(new LightPollerR(lightRightSensor));
		getLightPollerR().start();
		
		setLightLPoller(new LightPollerL(lightLeftSensor));
		getLightPollerL().start();
	}

	/*
	 * Get singleton method
	 * Call this to access the sensor manager
	 */
	public static SensorManager getInstance() throws OdometerExceptions {
		if (sharedManager == null) {
			sharedManager = new SensorManager();
		}
		return sharedManager;
	}

	/*
	 * Returns Ultrasonic Sensors distance (int)
	 */
	public int getDistance() {
		return this.getUsPoller().getDistance();
	}

	/*
	 * Returns Ultrasonic Sensors distance (double)
	 */

	/*
	 * Returns int if currently over a line 0 -> No line 1 -> Left Line 2 -> Right
	 * Line 3 -> Both Lines
	 */
	public int getLine() {
		int ret = 0;
		if (getLineL()) ret +=1; 
		if (getLineR()) ret +=2;
		return ret;
		
	}

	/*
	 * Returns true if currently over a line for the right sensor
	 */
	public boolean getLineR() {
		if (this.lightRightPoller.getDiff() < 0.07) {
			return false;
		}
		return true;
	}
	public boolean getLineL() {
		if (this.lightLeftPoller.getDiff() < 0.07) {
			return false;
		}
		return true;
	}

	/**
	 * Returns the Color Sensors current color
	 */
	public int getColor() {
		return this.colorPoller.getColorInt();
	}
	
	/**going to be used for distance measuring
	 *
	 */
	public double getEuclidColor() {
		return this.colorPoller.getEuclidColor();
	}

	/**
	 * @return the odometer
	 */
	public Odometer getOdometer() {
		return odometer;
	}

	/**
	 * @param odometer
	 *            the odometer to set
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
	
	public LightPollerR getLightPollerR() {
		return lightRightPoller;
	}
	public LightPollerL getLightPollerL() {
		return lightLeftPoller;
	}

	/**
	 * @param colorPoller
	 *            the colorPoller to set
	 */
	private void setColorPoller(ColorPoller colorPoller) {
		this.colorPoller = colorPoller;
	}
	
	private void setLightRPoller(LightPollerR lightPoller) {
		this.lightRightPoller = lightPoller;
	}
	
	private void setLightLPoller(LightPollerL lightPoller) {
		this.lightLeftPoller = lightPoller;
	}

	/**
	 * @return the lightSensor
	 */


	/**
	 * @param lightSensor
	 *            the lightSensor to set
	 */


	/**
	 * @return the usPoller
	 */
	public UltrasonicPoller getUsPoller() {
		return usPoller;
	}

	/**
	 * @param usPoller
	 *            the usPoller to set
	 */
	private void setUsPoller(UltrasonicPoller usPoller) {
		this.usPoller = usPoller;
	}

}
