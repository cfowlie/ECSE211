package lab5;

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

	// Color Sensor
	private static final Port colorPort = LocalEV3.get().getPort("S1");
	
	// Ultrasonic Sensor
	private static final Port usPort = LocalEV3.get().getPort("S2");

	// Light Sensor
	private static final Port lightPort = LocalEV3.get().getPort("S3");

	// Ultrasonic Sensor 2
	private static final Port distancePort = LocalEV3.get().getPort("S4");

	// Odometer
	private Odometer odometer;

	// Color Sensor
	private EV3ColorSensor colorSensor = new EV3ColorSensor(colorPort);
	private ColorPoller colorPoller;

	// Light Sensor
	private EV3ColorSensor lightSensor;

	// Ultrasonic Sensor
	private SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
	private SampleProvider usDistance = usSensor.getMode("Distance");
	private SensorModes distanceSensor = new EV3UltrasonicSensor(distancePort); // usSensor is the instance
	private SampleProvider distanceDistance = distanceSensor.getMode("Distance");// usDistance provides samples from
	private float[] usData = new float[usDistance.sampleSize()]; // usData is the buffer in which data are
	private UltrasonicPoller usPoller;
	private float[] distanceData = new float[distanceDistance.sampleSize()]; // usData is the buffer in which data are
	private UltrasonicPoller distancePoller;

	private SensorManager() throws OdometerExceptions {
		
		// Odometer
		this.setOdometer(Odometer.getOdometer());
		Thread odoThread = new Thread(getOdometer());
		odoThread.start();

		// Ultrasonic Sensor
		this.setUsPoller(new UltrasonicPoller(usDistance, usData));
		this.getUsPoller().start();
		
		this.setDistancePoller(new UltrasonicPoller(distanceDistance, distanceData));
		this.getDistancePoller().start();

		// Color Sensor
		setColorPoller(new ColorPoller(colorSensor));
		getColorPoller().start();

		// Light Sensor
		setLightSensor(new EV3ColorSensor(lightPort));
	}

	public static SensorManager getInstance() throws OdometerExceptions {
		if (sharedManager == null) {
			sharedManager = new SensorManager();
		}
		return sharedManager;
	}
	
	/*
	 * Returns Forward Ultrasonic Sensors distance
	 */
	public int getDistance() {
		return this.getUsPoller().getDistance();
	}
	
	/*
	 * Returns Side Ultrasonic Sensors distance
	 */
	public int getSideDistance() {
		return this.getDistancePoller().getDistance();
	}
	
	/*
	 * Returns true if currently over a line
	 */
	public boolean getLine() {
		if (this.lightSensor.getColorID() < 10) {
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
	public EV3ColorSensor getLightSensor() {
		return lightSensor;
	}

	/**
	 * @param lightSensor the lightSensor to set
	 */
	private void setLightSensor(EV3ColorSensor lightSensor) {
		this.lightSensor = lightSensor;
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
	
	public UltrasonicPoller getDistancePoller() {
		return distancePoller;
	}

	/**
	 * @param usPoller the usPoller to set
	 */
	public void setDistancePoller(UltrasonicPoller distancePoller) {
		this.distancePoller = distancePoller;
	}
	
	

}
