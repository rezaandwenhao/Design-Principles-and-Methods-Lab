/*
 * Navigation.java
 */
package ca.mcgill.ecse211.lab5;

import ca.mcgill.ecse211.odometer.Odometer;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class is used to drive the robot on the demo floor.
 */
public class Navigation {
  private static final int FORWARD_SPEED = 150;
  private static final int ROTATE_SPEED = 70;
  private static final double TILE_SIZE = 30.48;
  private EV3LargeRegulatedMotor motorL;
  private EV3LargeRegulatedMotor motorR;
  private double leftRadius;
  private double rightRadius;
  private double track;
  private Odometer odo;
    
  Navigation(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
	      double leftRadius, double rightRadius, double track, Odometer odo) {
	  this.motorL = leftMotor;
	  this.motorR = rightMotor;
	  this.leftRadius = leftRadius;
	  this.rightRadius = rightRadius;
	  this.track = track;
	  this.odo = odo;
  }

  
  /**
   * Rotates the robot to face the direction in which it needs to travel to
   * reach the point x,y.
   * Then rotates the wheels the exact distance from the current position to
   * the point x,y.
   * @param x in cm
   * @param y in cm
   */
  public void travelTo(double x, double y) {
	  
	  double currentPos[] = odo.getXYT();
	  
	  //difference in position
	  double deltaX = x-currentPos[0];
	  double deltaY = y-currentPos[1];
	  
	  // length of straight line from current position to desired position
	  double hypotenuse = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
	  
	  //angle from hypotenuse to X axis
	  if (deltaY == 0) deltaY = 0.0001;
	  double desiredAngle = Math.atan(deltaX/deltaY)*57.2957795131d;
	  
	  // adjust desired angle to desired position
	  if (deltaY < 0) {
		  desiredAngle += 180.0d;
	  }
	 
	  turnTo(desiredAngle);
	  
	  //move forwards to reach the point x,y.
	  moveForward(hypotenuse, true, FORWARD_SPEED);
  }
  
  /**
   * turns the robot from currentAngle to angle theta in the most efficient direction
   * 
   * @param theta: angle at which to turn clockwise
   * @param currentAngle: angle at which the robot is currently oriented
   */
  public void turnTo(double theta) {
	  double toRotate = theta - odo.getXYT()[2];
	  
	  if (toRotate > 180.0d) {
		  toRotate -= 360.0d;
	  } else if (toRotate < -180.0d) {
		  toRotate += 360.0d;
	  }
	  motorL.setSpeed(ROTATE_SPEED);
      motorR.setSpeed(ROTATE_SPEED);
      
	  motorL.rotate(convertAngle(leftRadius, track, toRotate), true);
      motorR.rotate(-convertAngle(rightRadius, track, toRotate), false);
  }
  
  /**
   * rotates the robot either clockwise or counter clockwise, for a specified angle. 
   * @param clockwise
   * @param angle
   * @param wait: if the robot should wait for the rotation to terminate.
   */
  public void rotate(boolean clockwise, int angle, boolean wait) {
	  motorL.setSpeed(ROTATE_SPEED);
      motorR.setSpeed(ROTATE_SPEED);
	  if (clockwise) {
		  motorL.rotate(convertAngle(leftRadius, track, angle), true);
	      motorR.rotate(-convertAngle(rightRadius, track, angle), !wait);
	  } else {
		  motorL.rotate(-convertAngle(leftRadius, track, angle), true);
	      motorR.rotate(convertAngle(rightRadius, track, angle), !wait);
	  }
  }
  
  /**
   * moves the robot forwards for a specified distance.
   * @param distance
   * @param wait: if the robot should wait for the rotation to terminate.
   */
  public void moveForward(double distance, boolean wait, int speed) {
	  motorL.setSpeed(speed);
	  motorR.setSpeed(speed);

      motorL.rotate(convertDistance(leftRadius, distance), true);
      motorR.rotate(convertDistance(rightRadius, distance), !wait);
  }
  

  public void moveBackward(double distance, boolean wait) {
	  motorL.setSpeed(FORWARD_SPEED);
	  motorR.setSpeed(FORWARD_SPEED);

      motorL.rotate(-convertDistance(leftRadius, distance), true);
      motorR.rotate(-convertDistance(rightRadius, distance), !wait);	
  }
  


  public void moveLeftMotor(int distance, boolean wait, int speed) {
	  motorL.setSpeed(speed);
	  
	  motorL.rotate(convertDistance(leftRadius, distance), !wait);
  }
  
  public void moveRightMotor(int distance, boolean wait, int speed) {
	  motorR.setSpeed(speed);
	  
	  motorR.rotate(convertDistance(rightRadius, distance), !wait);
  }

  /**
   * This method allows the conversion of a distance to the total rotation of each wheel need to
   * cover that distance.
   * 
   * @param radius
   * @param distance
   * @return
   */
  private static int convertDistance(double radius, double distance) {
    return (int) ((180.0 * distance) / (Math.PI * radius));
  }

  private static int convertAngle(double radius, double width, double angle) {
    return convertDistance(radius, Math.PI * width * angle / 360.0);
  }

  /**
   * stops the motors at the same time.
   */
  public void stop() {
	motorL.stop(true);
	motorR.stop(false);
  }


}
