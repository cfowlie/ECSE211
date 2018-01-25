
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class PController implements UltrasonicController {

	/* Constants */
	private static final int MOTOR_SPEED = 185;
	private static final int FILTER_OUT = 20;

	private final int bandCenter;
	private final int bandWidth;
	private int distance;
	private int filterControl;

	public PController(int bandCenter, int bandwidth) {
		this.bandCenter = bandCenter;
		this.bandWidth = bandwidth;
		this.filterControl = 0;
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

			// Attempt to hold distance at constant bandCenter
			if (distance <= 10) { // d < 10
				// Back up
				WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
				WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED);
				WallFollowingLab.rightMotor.backward();
				WallFollowingLab.leftMotor.backward();
			} else if (distance <= 20) { // Way too close to wall (Probably Corner)  10 < d < 20
				// Turn hard to the right
				WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
				WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED);
				WallFollowingLab.rightMotor.backward();
				WallFollowingLab.leftMotor.forward();
			} else if(distance < 30) { // 20 < d < 30
				// Turn more to the right
				int speedAdjustment = bandCenter + bandWidth - distance;
				WallFollowingLab.rightMotor.setSpeed((int)(MOTOR_SPEED));
				WallFollowingLab.leftMotor.setSpeed((int)(MOTOR_SPEED + speedAdjustment*0.95));
				WallFollowingLab.rightMotor.forward();
				WallFollowingLab.leftMotor.forward();
			} else if (distance <= 40) { // Good distance 30 < d < 40
				// Stay strait
				WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
				WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED);
				WallFollowingLab.rightMotor.forward();
				WallFollowingLab.leftMotor.forward();
			} else { // Too far from wall d > 40
				// Turn more to the left
				int speedAdjustment = distance - bandCenter + bandWidth;
				WallFollowingLab.rightMotor.setSpeed((int)(MOTOR_SPEED + speedAdjustment*0.95));
				WallFollowingLab.leftMotor.setSpeed((int)(MOTOR_SPEED));
				WallFollowingLab.rightMotor.forward();
				WallFollowingLab.leftMotor.forward();
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
