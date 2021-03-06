import lejos.hardware.motor.*;

public class BangBangController implements UltrasonicController {

  private final int bandCenter;
  private final int bandwidth;
  private final int motorLow;
  private final int motorHigh;
  private int distance;

  public BangBangController(int bandCenter, int bandwidth, int motorLow, int motorHigh) {
    // Default Constructor
    this.bandCenter = bandCenter;
    this.bandwidth = bandwidth;
    this.motorLow = motorLow;
    this.motorHigh = motorHigh;
    
  }

  @Override
  public void processUSData(int distance) {
    this.distance = distance;
    if(distance <= 2*bandCenter) {
    	WallFollowingLab.rightMotor.setSpeed(motorLow); //Way too close: do a spinning action
        WallFollowingLab.leftMotor.setSpeed(motorLow);
        WallFollowingLab.rightMotor.backward();
        WallFollowingLab.leftMotor.forward(); } 
    else if(distance >= 2*bandCenter + 3 && distance < 2*bandCenter + 5) {
    	WallFollowingLab.leftMotor.setSpeed(motorHigh); // a bit too close: slightly increase speed of left wheel
        WallFollowingLab.rightMotor.setSpeed(motorLow);
        WallFollowingLab.leftMotor.forward();
        WallFollowingLab.rightMotor.forward(); 
    } 
    else if(distance >= 2*bandCenter + 10){
        WallFollowingLab.leftMotor.setSpeed(motorLow); //Way too far, quickly turn back to course
        WallFollowingLab.rightMotor.setSpeed(motorHigh);
        WallFollowingLab.leftMotor.forward();
        WallFollowingLab.rightMotor.forward(); 
    }
    else {
	 	WallFollowingLab.leftMotor.setSpeed(motorHigh); //Sweet spot (occurs between distances of 25 and 30).
        WallFollowingLab.rightMotor.setSpeed(motorHigh);
        WallFollowingLab.leftMotor.forward();
        WallFollowingLab.rightMotor.forward(); 
    }
  }

  @Override
  public int readUSDistance() {
    return this.distance;
  }
}
