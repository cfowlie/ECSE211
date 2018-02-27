package light;

import lejos.hardware.sensor.EV3ColorSensor;

public class ColorPoller extends Thread implements Runnable {
	
	static float[] ColorID = new float[3];
	int offset;
	
	private EV3ColorSensor colorSensor;
	
	public static COLOR color;

	private static final long ODOMETER_PERIOD = 100;
	
	public enum COLOR{
        RED(0),
        BLUE(1),
        YELLOW(2),
        WHITE(3),
		DEFAULT(4);
		
		private final int value;

        COLOR(final int newValue) {
            value = newValue;
        }

        public int intValue() { return value; }
	}
	
	public ColorPoller(EV3ColorSensor colorSensor){
		this.setColorSensor(colorSensor);
	}
	
	public void run() {
		getColorSensor().setCurrentMode(2);

		long updateStart, updateEnd;
		while (true) {
			updateStart = System.currentTimeMillis();

			getColorSensor().fetchSample(ColorID, 0);

			if (ColorID[0] > 0.02 && ColorID[1] < 0.025 && ColorID[2] < 0.02) { // red detected
				// Sound.playTone(100, 300);
				color = COLOR.RED;

			} else if (ColorID[0] < 0.035 && ColorID[1] < 0.08 && ColorID[2] > 0.02) { // blue detected
				// Sound.playTone(430, 300);
				color = COLOR.BLUE;
			} else if (ColorID[0] > 0.04 && ColorID[1] > 0.03 && ColorID[2] < 0.04) { // yellow
				// Sound.playTone(1020, 300);
				color = COLOR.YELLOW;
			} else if (ColorID[0] > 0.07 && ColorID[1] > 0.07 && ColorID[2] > 0.05) { // white
				// Sound.playTone(1480, 300);
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
	
	
	public COLOR getColor() {
		return this.color;
	}
	
	public static int getColorInt() {
		return color.intValue();
	}

	/**
	 * @return the colorSensor
	 */
	private EV3ColorSensor getColorSensor() {
		return colorSensor;
	}

	/**
	 * @param colorSensor the colorSensor to set
	 */
	private void setColorSensor(EV3ColorSensor colorSensor) {
		this.colorSensor = colorSensor;
	}
	
}
