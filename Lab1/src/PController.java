

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class PController implements UltrasonicController {

  /* Constants */
  private static final int MOTOR_SPEED = 200;
  private static final int FILTER_OUT = 20;

  
  private final int bandCenter;
  private final int bandWidth;
  private int distance;
  private int filterControl;

  public PController(int bandCenter, int bandwidth) {
    this.bandCenter = bandCenter;
    this.bandWidth = bandwidth;
    this.filterControl = 0;

    WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED); // Initalize motor rolling forward
    WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
    WallFollowingLab.leftMotor.backward();
    WallFollowingLab.rightMotor.backward();
  }

  @Override
  public void processUSData(int distance) {

    // rudimentary filter - toss out invalid samples corresponding to null
    // signal.
    // (n.b. this was not included in the Bang-bang controller, but easily
    // could have).
    //
    if (distance >= 255 && filterControl < FILTER_OUT) {
      // bad value, do not set the distance var, however do increment the
      // filter value
      filterControl++;
    } else if (distance >= 255) {
      // We have repeated large values, so there must actually be nothing
      // there: leave the distance alone
      this.distance = distance;
    } else {
      // distance went below 255: reset filter and leave
      // distance alone.

      // Attempt to hold distance at constant 25
      if (distance <= 20) { // Way too close to wall (Probably Corner)
        // Turn more to the left
        int speedAdjustment = 20-distance;
        WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED + speedAdjustment);
        WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED + speedAdjustment);
        WallFollowingLab.rightMotor.forward();
        WallFollowingLab.leftMotor.backward();
      }
      else if (distance <= 25) { // Too close to wall
        // Turn more to the right
        int speedAdjustment = 25-distance;
        WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
        WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED + speedAdjustment);
        WallFollowingLab.rightMotor.backward();
        WallFollowingLab.leftMotor.backward();
      }
      else { // Too far from wall
        // Turn more to the left
        int speedAdjustment = distance-25;
        WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED + speedAdjustment);
        WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED);
        WallFollowingLab.rightMotor.backward();
        WallFollowingLab.leftMotor.backward();
      }

      filterControl = 0;
      this.distance = distance;
    }

  }


  @Override
  public int readUSDistance() {
    return this.distance;
  }

}
