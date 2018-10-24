package ca.mcgill.ecse211.lab5;

import ca.mcgill.ecse211.odometer.Odometer;
import lejos.hardware.Button;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.robotics.SampleProvider;

/**
 * This class is used to drive the robot on the demo floor. It contains methods
 * to drive robot to travel to a certain point or turn to desired angle
 * 
 * @author Eliott Bourachot, Wenhao Geng, Eden Ovadia
 */
public class Navigation extends Thread {
  private static final int FORWARD_SPEED = 150;
  private static final int ROTATE_SPEED = 70;
  private static final double TILE_SIZE = 30.48;
  private static final double COLOR_THRESHOLD = 17; // has been changed for Lab 5
  private static final int MEDIUM_MOTOR_SPEED = 30;
  private static final int CORRECTION_SPEED = 80;
 
  private EV3LargeRegulatedMotor motorL;
  private EV3LargeRegulatedMotor motorR;
  private EV3MediumRegulatedMotor mediumM;
  private double leftRadius;
  private double rightRadius;
  private double track;
  private Odometer odo;
  
  private SampleProvider lightMeanL;
  public float[] lightDataL;
  private SampleProvider lightMeanR;
  public float[] lightDataR;
  
  private final int LIGHT_Y_OFFSET = 9; // Y offset to account for the light sensor not being in the middle (cm)
  private final int LIGHT_X_OFFSET = 10; // X offset to account for the light sensor not being in the middle (cm)

  Navigation(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, double leftRadius, double rightRadius,
      double track, Odometer odo, EV3MediumRegulatedMotor mediumM, SampleProvider lightMean1, float[] lightData1,
      SampleProvider lightMean2, float[] lightData2) {
    this.motorL = leftMotor;
    this.motorR = rightMotor;
    this.leftRadius = leftRadius;
    this.rightRadius = rightRadius;
    this.track = track;
    this.odo = odo;
    this.mediumM = mediumM;
    this.lightMeanL = lightMean1;
    this.lightDataL = lightData1;
    this.lightMeanR = lightMean2;
    this.lightDataR = lightData2;
  }

  
  public void run() {
    int[] settings = {1,1,4,4,0,0}; // LLx = 0, LLy = 0, URx = 0, URy = 0, TR = 0, SC = 0;
    int LLx = settings[0], LLy = settings[1], URx = settings[2], URy = settings[3];
    
    // travel to the left corner of searching area
    travelTo(LLx*TILE_SIZE, LLy*TILE_SIZE);
    Button.waitForAnyPress();
    travelTo((LLx+0.6)*TILE_SIZE, (LLy+0.6)*TILE_SIZE);

    
    /*
     * We will alternatively perform a set of movements: left side (of the searching
     * area) to right side (or right side to left side) crossing, move up one tile,
     * and then rotate the light sensor 180 degrees to the other side of the robot,
     * until all the intersections within the searching area is checked
     */
    int height = URy - LLy - 1; //the total amount of the set of movements need to be done
    int currentY = 0;  //current counter of the set of movements has been done
    boolean right = true;   //indicate whether to do left side to right side crossing or the other way
    while (currentY < height) {
      currentY++;
      if (right) {
        right = false;  //change to false so the next iteration it will do right side to left side crossing
        turnTo(90);
        //using the first vertical black line to correct odo
        updateOdo((LLx+1)*TILE_SIZE-LIGHT_X_OFFSET, odo.getXYT()[1], 90);
        moveForward((URx-LLx-1)*TILE_SIZE-LIGHT_X_OFFSET, true, FORWARD_SPEED);
        turnTo(0);
        //x stays unchanged, update y
        updateOdo(odo.getXYT()[0], (LLy+currentY)*TILE_SIZE-LIGHT_Y_OFFSET, 0);
        moveForward(TILE_SIZE-LIGHT_Y_OFFSET, true, FORWARD_SPEED);
        turnMediumMotor(-120);
        
      } else {
        right = true;
        turnTo(-90);
        updateOdo((URx-1)*TILE_SIZE+LIGHT_X_OFFSET, odo.getXYT()[1], -90);
        moveForward((URx-LLx-1)*TILE_SIZE-LIGHT_X_OFFSET, true, FORWARD_SPEED);
        turnTo(0);
        updateOdo(odo.getXYT()[0], (LLy+currentY)*TILE_SIZE-LIGHT_Y_OFFSET, 0);
        moveForward(TILE_SIZE-LIGHT_Y_OFFSET, true, FORWARD_SPEED);
        turnMediumMotor(120);
      }
    }
    
    //travel to the upper right corner of the searching area
    travelTo(URx*TILE_SIZE, URy*TILE_SIZE);
    
  }
  
  /**
   * Rotates the robot to face the direction in which it needs to travel to
   * reach the point x,y.
   * Then rotates the wheels the exact distance from the current position to
   * the point x,y.
   * 
   * 
   * @param x in cm
   * @param y in cm
   * @param onlyTurntoDest if true, then only turn to dest but do not move forward
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
   */
  private static int convertDistance(double radius, double distance) {
    return (int) ((180.0 * distance) / (Math.PI * radius));
  }

  /**
   * This method allows the conversion of an angle to the total rotation of each wheel need to
   * turn that angle.
   * 
   * @param radius
   * @param width
   * @param angle
   */
  private static int convertAngle(double radius, double width, double angle) {
    return convertDistance(radius, Math.PI * width * angle / 360.0);
  }

  /**
   * stops the motors at the same time.
   */
  public void stopMotors() {
	motorL.stop(true);
	motorR.stop(false);
  }

  /**
   * turn the medium motor a desired angle with predefined motor speed
   * 
   * @param angle
   */
  public void turnMediumMotor(int angle) {
    mediumM.setSpeed(MEDIUM_MOTOR_SPEED);
    mediumM.rotate(angle, false);
    mediumM.stop();
  }
  
  /**
   * use two light sensors to bring the robot to face either perfect horizatonally
   * or vertically, and update the odo data
   * 
   * @param x, the x value to update odo to
   * @param y, the y value to update odo to
   * @param theta, the theta value to update odo to
   */
  public void updateOdo(double x, double y, double theta) {
    // Read both sensors once at first
    lightMeanL.fetchSample(lightDataL, 0); // acquire data
    int lightL = (int) (lightDataL[0] * 100.0); // extract from buffer, cast to int
    
    lightMeanR.fetchSample(lightDataR, 0); // acquire data
    int lightR = (int) (lightDataR[0] * 100.0); // extract from buffer, cast to int
    
    // Move forwards until first sensor hits line
    moveForward(TILE_SIZE, false, CORRECTION_SPEED);
    while (lightL > COLOR_THRESHOLD && lightR > COLOR_THRESHOLD) { // move forward until you hit a black band
        lightMeanL.fetchSample(lightDataL, 0); // acquire data
        lightL = (int) (lightDataL[0] * 100.0); // extract from buffer, cast to int
        lightMeanR.fetchSample(lightDataR, 0); // acquire data
        lightR = (int) (lightDataR[0] * 100.0); // extract from buffer, cast to int
    }
    stopMotors();
    
    // Move whichever sensor didn't hit line until it hits the line
    if (lightL > COLOR_THRESHOLD) {
        moveLeftMotor(3, false, 30);
        while (lightL > COLOR_THRESHOLD) {
            lightMeanL.fetchSample(lightDataL, 0); // acquire data
            lightL = (int) (lightDataL[0] * 100.0); // extract from buffer, cast to int
        }
        moveLeftMotor(-5, false, 30);
        while (lightL > COLOR_THRESHOLD) {
            lightMeanL.fetchSample(lightDataL, 0); // acquire data
            lightL = (int) (lightDataL[0] * 100.0); // extract from buffer, cast to int
        }
        stopMotors();
    } else if (lightR > COLOR_THRESHOLD) {
        moveRightMotor(3, false, 30);
        while (lightR > COLOR_THRESHOLD) {
            lightMeanR.fetchSample(lightDataR, 0); // acquire data
            lightR = (int) (lightDataR[0] * 100.0); // extract from buffer, cast to int
        }
        moveRightMotor(-5, false, 30);
        while (lightR > COLOR_THRESHOLD) {
            lightMeanR.fetchSample(lightDataR, 0); // acquire data
            lightR = (int) (lightDataR[0] * 100.0); // extract from buffer, cast to int
        }
        stopMotors();
    }
    odo.setX(x);
    odo.setY(y);
    odo.setTheta(theta);
  }
  
}
