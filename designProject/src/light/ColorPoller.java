package light;

import lejos.hardware.sensor.EV3ColorSensor;

/**
 * This class is responsible for using the front facing light sensor to detect
 * and return the color of a block, and for using the Euclidean color the
 * determine how far the robot is away from a block.
 * 
 * @author Connor Fowlie
 * @author Lucas Bluethner
 * @version Final 1.0.5
 *
 */
public class ColorPoller extends Thread implements Runnable {

	static float[] ColorID = new float[3];
	int offset;

	private EV3ColorSensor colorSensor;

	public static COLOR color;

	public static double euclidColor;

	private static final long ODOMETER_PERIOD = 100;

	public enum COLOR {
		RED(1), BLUE(2), YELLOW(3), WHITE(4), DEFAULT(0);

		private final int value;

		COLOR(final int newValue) {
			value = newValue;
		}

		public int intValue() {
			return value;
		}
	}

	/**
	 * @param colorSensor
	 */
	public ColorPoller(EV3ColorSensor colorSensor) {
		this.setColorSensor(colorSensor);
	}

	/**
	 * This method used RGB mode of the front facing light sensor to detect the
	 * color of a block.
	 */
	public void run() {
		getColorSensor().setCurrentMode(2);

		long updateStart, updateEnd;
		while (true) {
			updateStart = System.currentTimeMillis();

			getColorSensor().fetchSample(ColorID, 0);

			euclidColor = Math.sqrt(Math.pow(ColorID[0], 2) + Math.pow(ColorID[1], 2) + Math.pow(ColorID[2], 2));

			if (ColorID[0] > 0.02 && ColorID[1] < 0.025 && ColorID[2] < 0.02) { // red detected
				color = COLOR.RED;
			} else if (ColorID[0] < 0.035 && ColorID[1] < 0.08 && ColorID[2] > 0.02) { // blue detected
				color = COLOR.BLUE;
			} else if (ColorID[0] > 0.04 && ColorID[1] > 0.03 && ColorID[2] < 0.04) { // yellow
				color = COLOR.YELLOW;
			} else if (ColorID[0] > 0.07 && ColorID[1] > 0.07 && ColorID[2] > 0.05) { // white
				color = COLOR.WHITE;
			} else {
				color = COLOR.DEFAULT;
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

	/**
	 * Returns the current color detected by the front facing light sensor.
	 * 
	 * @return color detected by the front facing light sensor
	 */
	public COLOR getColor() {
		return this.color;
	}

	/**
	 * Returns the current color detected by the front facing light sensor as an
	 * int.
	 * 
	 * @return color detected as an int
	 */
	public int getColorInt() {
		return color.intValue();
	}

	/**
	 * Returns the Euclidean Color from the front facing light sensor.
	 * 
	 * @return Euclidian color
	 */
	public double getEuclidColor() {
		return euclidColor;
	}

	/**
	 * @return the colorSensor
	 */
	private EV3ColorSensor getColorSensor() {
		return colorSensor;
	}

	/**
	 * @param colorSensor
	 *            the colorSensor to set
	 */
	private void setColorSensor(EV3ColorSensor colorSensor) {
		this.colorSensor = colorSensor;
	}

}
