

import java.text.DecimalFormat;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.sensor.EV3ColorSensor;

/**
 * This class is used to display the content of the odometer variables (x, y, Theta)
 */
public class Display implements Runnable {

 
  private TextLCD lcd;
  
  private final long DISPLAY_PERIOD = 25;
  private long timeout = Long.MAX_VALUE;

  /**
   * This is the class constructor
   * 
   * @param odoData
   * @throws OdometerExceptions 
   */
  public Display(TextLCD lcd)  {
    
    this.lcd = lcd;
  }

  /**
   * This is the overloaded class constructor
   * 
   * @param odoData
   * @throws OdometerExceptions 
   */
  public Display(TextLCD lcd, long timeout) {
   
    this.timeout = timeout;
    this.lcd = lcd;
  }
  
  String[] walls = {"Red","Blue","Yellow","White","Void"};
  
  private String wallType(int i) {
	  return walls[i];
  }

  public void run() {
    
    lcd.clear();
    
    long updateStart, updateEnd;

    long tStart = System.currentTimeMillis();
    do {
      updateStart = System.currentTimeMillis();

      // Retrieve x, y and Theta information
     
      
      // Print x,y, and theta information
      DecimalFormat numberFormat = new DecimalFormat("######0.0000");
      lcd.drawString("R: " + numberFormat.format(ColorIdentification.ColorID[0]), 0, 0);
      lcd.drawString("G: " + numberFormat.format(ColorIdentification.ColorID[1]), 0, 1);
      lcd.drawString("B: " + numberFormat.format(ColorIdentification.ColorID[2]), 0, 2);
      lcd.drawString("Wall Type:" + wallType(ColorIdentification.wall), 0, 4);
   
      
      // this ensures that the data is updated only once every period
      updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < DISPLAY_PERIOD) {
        try {
          Thread.sleep(DISPLAY_PERIOD - (updateEnd - updateStart));
        } catch (InterruptedException e) {
          e.printStackTrace();
        }
        lcd.clear(4);
      }
    } while ((updateEnd - tStart) <= timeout);

  }

}
