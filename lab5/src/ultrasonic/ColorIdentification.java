
package ultrasonic;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.sensor.EV3ColorSensor;

public class ColorIdentification {

	static float[] ColorID = new float[3];
	int offset;

	public static int wall;

	private static final long ODOMETER_PERIOD = 100;

	private static final EV3ColorSensor colorSensor = new EV3ColorSensor(LocalEV3.get().getPort("S1"));

	public static void colorID() throws InterruptedException {

		colorSensor.setCurrentMode(2);

		long updateStart, updateEnd;
		while (true) {
			updateStart = System.currentTimeMillis();

			colorSensor.fetchSample(ColorID, 0);

			if (ColorID[0] > 0.02 && ColorID[1] < 0.025 && ColorID[2] < 0.02) { // red detected
				// Sound.playTone(100, 300);
				wall = 0;

			} else if (ColorID[0] < 0.035 && ColorID[1] < 0.08 && ColorID[2] > 0.02) { // blue detected
				// Sound.playTone(430, 300);
				wall = 1;
			} else if (ColorID[0] > 0.04 && ColorID[1] > 0.03 && ColorID[2] < 0.04) { // yellow
				// Sound.playTone(1020, 300);
				wall = 2;
			} else if (ColorID[0] > 0.07 && ColorID[1] > 0.07 && ColorID[2] > 0.05) { // white
				// Sound.playTone(1480, 300);
				wall = 3;
			} else {
				wall = 4;
			}
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

}
