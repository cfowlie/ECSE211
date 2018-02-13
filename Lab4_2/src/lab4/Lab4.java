package lab4;

import odometer.*;
import lejos.hardware.sensor.*;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.robotics.SampleProvider;

import lejos.hardware.Button;

public class Lab4 {

  // Parameters: adjust these for desired performance

  private static final int bandCenter = 20; // Offset from the wall (cm)
  private static final int bandWidth = 10; // Width of dead band (cm)
  private static final int motorLow = 150; // Speed of slower rotating wheel (deg/sec)
  private static final int motorHigh = 300; // Speed of the faster rotating wheel (deg/seec)
	public static final double WHEEL_RAD = 2.095;
	public static final double TRACK = 8.5;
  
  
  private static final TextLCD lcd = LocalEV3.get().getTextLCD();
  private static final Port usPort = LocalEV3.get().getPort("S2");
  public static final EV3LargeRegulatedMotor leftMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
  public static final EV3LargeRegulatedMotor rightMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));

  // Main entry point - instantiate objects used and set up sensor

  public static void main(String[] args) throws OdometerExceptions {

	  
	  
    int option = 0;
    Printer.printMainMenu(); // Set up the display on the EV3 screen
    while (option == 0) // and wait for a button press. The button
      option = Button.waitForAnyPress(); // ID (option) determines what type of control to use

    // Setup controller objects

    FallingEdge fall =
        new FallingEdge(bandCenter, bandWidth, motorHigh);

    //PController pController = new PController(bandCenter, bandWidth);

    // Setup ultrasonic sensor
    // There are 4 steps involved:
    // 1. Create a port object attached to a physical port (done already above)
    // 2. Create a sensor instance and attach to port
    // 3. Create a sample provider instance for the above and initialize operating mode
    // 4. Create a buffer for the sensor data
    Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD); 
    // OdometryCorrection odometryCorrection = OdometryCorrection.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);                                                               
     Display odometryDisplay = new Display(lcd); // No need to change
    
    
    @SuppressWarnings("resource") // Because we don't bother to close this resource
    SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
    SampleProvider usDistance = usSensor.getMode("Distance"); // usDistance provides samples from
                                                              // this instance
    float[] usData = new float[usDistance.sampleSize()]; // usData is the buffer in which data are
                                                         // returned

    // Setup Printer
    // This thread prints status information in the background
    Printer printer = null;

    // Setup Ultrasonic Poller // This thread samples the US and invokes
    UltrasonicPoller usPoller = null; // the selected controller on each cycle

    // Depending on which button was pressed, invoke the US poller and printer with the
    // appropriate constructor.
    
    int choice = 0;

    switch (option) {
      case Button.ID_LEFT: // Bang-bang control selected
        usPoller = new UltrasonicPoller(usDistance, usData, fall);
        
        printer = new Printer(option, fall);
        
        choice = 0;
        
        break;
     // case Button.ID_RIGHT: // Proportional control selected
      //  usPoller = new UltrasonicPoller(usDistance, usData, pController);
      //  printer = new Printer(option, pController);
      //choice = 1;
        // break;

        
      default:
        System.out.println("Error - invalid button"); // None of the above - abort
        System.exit(-1);
        break;
    }

 // Start the poller and printer threads
    usPoller.start();
    printer.start();
    
    if(choice == 0) {
    	
    
    	
    	Thread odoThread = new Thread(odometer);
        odoThread.start();
        Thread odoDisplayThread = new Thread(odometryDisplay);
        odoDisplayThread.start();
    	
    	
    	(new Thread() {
            public void run() {
            	FallingEdge.Fall();
            }
          }).start();
    }
    

    // Wait here forever until button pressed to terminate wallfollower
    Button.waitForAnyPress();
    System.exit(0);

  }
}
