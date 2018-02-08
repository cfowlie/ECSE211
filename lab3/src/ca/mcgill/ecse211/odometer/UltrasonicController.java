package ca.mcgill.ecse211.odometer;


public interface UltrasonicController {

  public void processUSData(int distance);

  public int readUSDistance();
}
