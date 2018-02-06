// Lab2.java
package ca.mcgill.ecse211.lab2;

import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Lab3 {

  // Motor Objects, and Robot related parameters
  private static final EV3LargeRegulatedMotor leftMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
  private static final EV3LargeRegulatedMotor rightMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
  private static final TextLCD lcd = LocalEV3.get().getTextLCD();
  public static final double WHEEL_RAD = 2.095;
  public static final double TRACK = 9.1;

  public static void main(String[] args) throws OdometerExceptions {

    int buttonChoice;

    // Odometer related objects
    Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD); 
   // OdometryCorrection odometryCorrection = OdometryCorrection.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);                                                               
    Display odometryDisplay = new Display(lcd); // No need to change


    do {
      // clear the display
      lcd.clear();

      // ask the user whether the motors should drive in a square or float
      lcd.drawString("Select map      ", 0, 0);
      lcd.drawString(" Up for 1       ", 0, 1);
      lcd.drawString(" Down for 2     ", 0, 2);
      lcd.drawString(" Right for 3    ", 0, 3);
      lcd.drawString(" Left for 4     ", 0, 4);

      buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
    } while (buttonChoice != Button.ID_DOWN && buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT && buttonChoice != Button.ID_UP);

    if (buttonChoice == Button.ID_UP) {
    
      lcd.clear();
    
      Thread odoThread = new Thread(odometer);
      odoThread.start();
      Thread odoDisplayThread = new Thread(odometryDisplay);
      odoDisplayThread.start();
      
      
      (new Thread() {
          public void run() {
            Navigation.drive(leftMotor, rightMotor, WHEEL_RAD, WHEEL_RAD, TRACK, 0);
          }
        }).start();
      

    } else if (buttonChoice == Button.ID_DOWN) {
      // clear the display
      lcd.clear();

      Thread odoThread = new Thread(odometer);
      odoThread.start();
      Thread odoDisplayThread = new Thread(odometryDisplay);
      odoDisplayThread.start();
      
      (new Thread() {
          public void run() {
            Navigation.drive(leftMotor, rightMotor, WHEEL_RAD, WHEEL_RAD, TRACK, 1);
          }
        }).start();
      

    } else if (buttonChoice == Button.ID_RIGHT ) {
    	
      lcd.clear();

      Thread odoThread = new Thread(odometer);
      odoThread.start();
      Thread odoDisplayThread = new Thread(odometryDisplay);
      odoDisplayThread.start();
    	
    

      // spawn a new Thread to avoid SquareDriver.drive() from blocking
      (new Thread() {
        public void run() {
          Navigation.drive(leftMotor, rightMotor, WHEEL_RAD, WHEEL_RAD, TRACK,2);
        }
      }).start();
    } else if (buttonChoice == Button.ID_LEFT ) {
    	
        lcd.clear();

        Thread odoThread = new Thread(odometer);
        odoThread.start();
        Thread odoDisplayThread = new Thread(odometryDisplay);
        odoDisplayThread.start();
      	
      

        // spawn a new Thread to avoid SquareDriver.drive() from blocking
        (new Thread() {
          public void run() {
            Navigation.drive(leftMotor, rightMotor, WHEEL_RAD, WHEEL_RAD, TRACK,3);
          }
        }).start();
      }

    Button.waitForAnyPress();
    System.exit(0);
  }
}
