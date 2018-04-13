package ultrasonic;

import lejos.robotics.SampleProvider;

/**
 * This class is where the Ultrasonic data is read from the sensor and stored.
 * 
 * @author Connor Fowlie
 */
public class UltrasonicPoller extends Thread implements Runnable {
	private SampleProvider us;
	private float[] usData;
	private double distance;

	public UltrasonicPoller(SampleProvider us, float[] usData) {
		this.us = us;
		this.usData = usData;
	}

	/*
	 * Sensors now return floats using a uniform protocol. Need to convert US result
	 * to an integer [0,255] (non-Javadoc)
	 * 
	 * @see java.lang.Thread#run()
	 */
	public void run() {
		double distance;
		while (true) {
			us.fetchSample(usData, 0); // acquire data
			distance = (int) (usData[0] * 100.0); // extract from buffer, cast to int
			this.distance = distance;
			try {
				Thread.sleep(25);
			} catch (Exception e) {
			} // Poor man's timed sampling
		}
	}

	/**
	 * @return distance read by the ultrasonic sensor as an int.
	 */
	public int getDistance() {
		return (int) this.distance;
	}

	/**
	 * @return distance read by the ultrasonic sensor as an double.
	 */
	public double getDistanceD() {
		return this.distance;
	}

}
