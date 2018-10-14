package ca.mcgill.ecse211.lab5;

import ca.mcgill.ecse211.lab5.Odometer;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

/**
 * This class continues the localization from UltrasonicLocalization class.
 * The thread will bring the robot close to (0,0), and then it will start to turn 
 * 360 degrees to note down all the angles needed to calculate its current coordinate 
 * and theta with respect to (0,0) and y axis. Thus, it can travel to (0,0)
 * and turn the robot to y axis facing away from the way better than UltrasonicLocalization
 * 
 * @author Wenhao Geng, Edgar Chang
 *
 */
public class ColorSensorLocalization extends Thread {
  private EV3LargeRegulatedMotor leftMotor = Lab5.leftMotor;
  private EV3LargeRegulatedMotor rightMotor = Lab5.rightMotor;
  private static final double WHEEL_RAD = Lab5.WHEEL_RAD;
  private static final double TRACK = Lab5.TRACK;
  
  private Odometer odo;
  private Navigation nav;

  private static final int FORWARD_SPEED = 100;
  private static final int TURNING_SPEED = 100;
  private static final int ACCELERATION = 200;
  private static final double CENTERTOSENSOR = 5;
  private static final double COLOR_THRESHOLD = 0.28;
  private static final int CORRECTION = 5; // correction apply to the final turn, determined by our test

  private double[] xyMinusPlus = new double[4];

  private SampleProvider colorSampleP;
  private float[] sampleColor;
  
  public ColorSensorLocalization(Navigation nav, Odometer odo, SampleProvider lightMean, float[] lightData) {
      this.nav = nav;
      this.odo = odo;
      this.colorSampleP = lightMean;
      this.sampleColor = lightData;
  }

  public void run() {
    Button.waitForAnyPress();
    leftMotor.stop();
    rightMotor.stop();

    // Set the acceleration so the motor will accelerate gradually
    leftMotor.setAcceleration(ACCELERATION);
    rightMotor.setAcceleration(ACCELERATION);

    // Get the robot closer to the origin ensure that the light localization can touch 4 grid lines
    getCloserToOrigin();

    // Rotate, note angle down when detecting any grid line
    doLightLocalization();

    // Do the calculation with angles recorded and travel to (0,0)
    double xTheta = xyMinusPlus[1] - xyMinusPlus[3]; //xPlus - xMinus
    double yTheta = xyMinusPlus[2] - xyMinusPlus[0]; //yMinus - yPlus

    double x = -CENTERTOSENSOR * Math.cos((yTheta / 2) * Math.PI / 180);
    double y = -CENTERTOSENSOR * Math.cos((xTheta / 2) * Math.PI / 180);
    double deltaTheta = 270 - xyMinusPlus[3] + (yTheta / 2);

    odo.setX(x);
    odo.setY(y);
    odo.setTheta(odo.getXYT()[2] + deltaTheta);
 
    nav.travelTo(0, 0);

    // let robot turn back to 0 degree
    leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, odo.getXYT()[2]+CORRECTION), true);
    rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, odo.getXYT()[2]+CORRECTION), false);
  }
  
  /**
   * This method gets the robot closer to the origin to ensure that the light
   * localization can touch 4 grid lines. After ultrasonic localization, the robot
   * is roughly at 0 degree facing away from the wall. We let the robot travel
   * forward until it detects a grid line. And then, it moves backward for 20cm.
   * By doing so, the robot will be reasonably close to the x axis. We turn right
   * the robot 90 degrees and do the same routine again. The robot will end up
   * being close to y axis.
   */
  private void getCloserToOrigin() {

    // The robot is moving forward until a grid line is detected
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);

    leftMotor.forward();
    rightMotor.forward();

    while (leftMotor.isMoving() && rightMotor.isMoving()) {
      // fetching the values from the color sensor
      colorSampleP.fetchSample(sampleColor, 0);
      float value = sampleColor[0];
      if (value < COLOR_THRESHOLD) {
        // When detected black line, the method is returned
        Sound.beep();
        break;
      }
    }

    // Reverse 20cm
    nav.moveForward(2, true);

    leftMotor.setSpeed(TURNING_SPEED);
    rightMotor.setSpeed(TURNING_SPEED);

    // right turn 90 degrees
    leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 90), true);
    rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 90), false);

    // Move forward again until a grid line is detected
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);

    leftMotor.forward();
    rightMotor.forward();

    while (leftMotor.isMoving() && rightMotor.isMoving()) {
      // fetching the values from the color sensor
      colorSampleP.fetchSample(sampleColor, 0);
      float value = sampleColor[0];
      if (value < COLOR_THRESHOLD) {
        // When detected black line, the method is returned
        Sound.beep();
        break;
      }
    }

    //reverse 20cm
    nav.moveForward(2, true);

    leftMotor.setSpeed(TURNING_SPEED);
    rightMotor.setSpeed(TURNING_SPEED);

    // left turn 90 degrees
    leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 90), true);
    rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 90), false);
  }

  /**
   * This method renders the robot turn 360 degrees clockwise and then store the
   * theta on odometer everytime (should be 4 times in total) it detects a grid line
   */
  private void doLightLocalization() {
    int lineCtr = 0;
    
    leftMotor.setSpeed(TURNING_SPEED);
    rightMotor.setSpeed(TURNING_SPEED);

    // rotate the robot 360 degrees clockwise
    leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 360), true);
    rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 360), true);

    while (leftMotor.isMoving() && rightMotor.isMoving()) {
      // fetching the values from the color sensor
      colorSampleP.fetchSample(sampleColor, 0);
      float value = sampleColor[0];

      if (value < COLOR_THRESHOLD) {
        Sound.beep();
        // store the odometer theta to xyMinusPlus where
        // yPlus = xyMinusPlus[0], xPlus = xyMinusPlus[1], yMinus = xyMinusPlus[2], xMinus = xyMinusPlus[3]
        if (lineCtr == 4) {
          // do nothing
        } else {
          xyMinusPlus[lineCtr] = odo.getXYT()[2];
          lineCtr++;
        }
      }

    }
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
}