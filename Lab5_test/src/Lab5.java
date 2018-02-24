// Lab2.java

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Lab5 {

  // Motor Objects, and Robot related parameters
  private static final EV3LargeRegulatedMotor leftMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
  private static final EV3LargeRegulatedMotor rightMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
  private static final TextLCD lcd = LocalEV3.get().getTextLCD();
  public static final double WHEEL_RAD = 2.095;
  public static final double TRACK = 9.17;

  public static void main(String[] args) {

    int buttonChoice;

    // Odometer related objects
                                                           
    Display odometryDisplay = new Display(lcd); // No need to change


    do {
      // clear the display
      lcd.clear();
      lcd.clear();

      // ask the user whether odometery correction should be run or not
      lcd.drawString("< Left | Right >", 0, 0);
      lcd.drawString(" Check |        ", 0, 1);
      lcd.drawString(" color |        ", 0, 2);
      lcd.drawString("       |        ", 0, 3);
      lcd.drawString("       |        ", 0, 4);

      buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
    } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);



      // Display changes in position as wheels are (manually) moved
      
     
      Thread odoDisplayThread = new Thread(odometryDisplay);
      odoDisplayThread.start();


      // clear the display
     
 // Record choice (left or right press)

      // Start correction if right button was pressed
      if (buttonChoice == Button.ID_LEFT) {
    	  (new Thread() {
    	        public void run() {
    	          try {
					ColorIdentification.colorID();
				} catch (InterruptedException e) {
					
				}
    	        }
    	      }).start();
    	    }
    	  
      
      
      
    Button.waitForAnyPress();
    System.exit(0);
  }
}
