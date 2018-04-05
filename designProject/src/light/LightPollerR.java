package light;

import lejos.hardware.sensor.EV3ColorSensor;

public class LightPollerR extends Thread implements Runnable {
	
	static float[] RedID = new float[3];
	int offset;
	
	private EV3ColorSensor lightSensor;
	
	public static float color;

	private static final long ODOMETER_PERIOD = 100;
	

	
	public LightPollerR(EV3ColorSensor lightSensor){
		this.setColorSensor(lightSensor);
	}
	
	public void run() {
		getLightSensor().setCurrentMode(1);

		long updateStart, updateEnd;
		while (true) {
			updateStart = System.currentTimeMillis();

			getLightSensor().fetchSample(RedID, 0);

			color = RedID[0];
			
			
			updateEnd = System.currentTimeMillis();
			if (updateEnd - updateStart < ODOMETER_PERIOD) {
				try {
					Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
				} catch (InterruptedException e) {
					// there is nothing to be done
				}
			}
		}
	}
	
	
	public float getColor() {
		return this.color;
	}
	

	/**
	 * @return the colorSensor
	 */
	private EV3ColorSensor getLightSensor() {
		return lightSensor;
	}

	/**
	 * @param colorSensor the colorSensor to set
	 */
	private void setColorSensor(EV3ColorSensor lightSensor) {
		this.lightSensor = lightSensor;
	}
	
}
