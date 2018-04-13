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

/**
 * The SensorManager manages all the methods responsible for interacting with
 * the ultrasonic sensor and light sensors.
 * 
 * @author Connor Fowlie, David Castonguay, Lucas Bluethner
 * @version Final 1.0.5
 * 
 */
public class SensorManager {

	/** Singleton Object */
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

	/**
	 * Init method sets up instances of sensors on threads and starts.
	 * 
	 * @throws OdometerExceptions
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

	/**
	 * Get singleton method, call this to access the sensor manager.
	 * 
	 * @return sharedManager
	 * @throws OdometerExceptions
	 */
	public static SensorManager getInstance() throws OdometerExceptions {
		if (sharedManager == null) {
			sharedManager = new SensorManager();
		}
		return sharedManager;
	}

	/**
	 * Returns Ultrasonic Sensors distance in cm
	 * 
	 * @return integer ultrasonic sensor distance
	 */
	public int getDistance() {
		return this.getUsPoller().getDistance();
	}

	/**
	 * Returns int if currently over a line: 0 = No line, 1 = Left Line, 2 = Right
	 * Line, 3 = Both Lines.
	 * 
	 * @return 0 if neither sensor is over a line, 1 if left sensor is over a line,
	 *         2 if right sensor is over a line, and 3 if both sensor are over a
	 *         line
	 */
	public int getLine() {
		int ret = 0;
		if (getLineL())
			ret += 1;
		if (getLineR())
			ret += 2;
		return ret;
	}

	/**
	 * Returns true if the right light sensor is currently over a line.
	 * 
	 * @return true if a line is detected; false otherwise.
	 */
	public boolean getLineR() {
		if (this.lightRightPoller.getDiff() < 0.07) {
			return false;
		}
		return true;
	}

	/**
	 * Returns true if the left light sensor is currently over a line.
	 * 
	 * @return true if a line is detected; false otherwise.
	 */
	public boolean getLineL() {
		if (this.lightLeftPoller.getDiff() < 0.07) {
			return false;
		}
		return true;
	}

	/**
	 * Returns the front facing light sensors current color as an int.
	 * 
	 * @return color from the collorPoller as an integer value.
	 */
	public int getColor() {
		return this.colorPoller.getColorInt();
	}

	/**
	 * Uses the Euclidean color from the forward facing light sensor to detect
	 * distance from a block.
	 * 
	 * @return Euclidean color from the colorPoller
	 */
	public double getEuclidColor() {
		return this.colorPoller.getEuclidColor();
	}

	/**
	 * Returns the current instance of the Odometer.
	 * 
	 * @return odometer instance
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
	 * Returns the current instance of the ColorPoller.
	 * 
	 * @return the colorPoller
	 */
	public ColorPoller getColorPoller() {
		return colorPoller;
	}

	/**
	 * Returns the current instance of LightPollerR.
	 * 
	 * @return lightRightPoller
	 */
	public LightPollerR getLightPollerR() {
		return lightRightPoller;
	}

	/**
	 * Returns the current instance of LightPollerL.
	 * 
	 * @return lightLefttPoller
	 */
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
