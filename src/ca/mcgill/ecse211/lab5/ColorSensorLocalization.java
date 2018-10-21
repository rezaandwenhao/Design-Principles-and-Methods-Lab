package ca.mcgill.ecse211.lab5;

import ca.mcgill.ecse211.odometer.Odometer;
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

  
  private Odometer odo;
  private Navigation nav;

  private final int LIGHT_Y_OFFSET = 9; // Y offset to account for the light sensor not being in the middle (cm)
  private final int LIGHT_X_OFFSET = 10; // X offset to account for the light sensor not being in the middle (cm)
  private static final double COLOR_THRESHOLD = 20; // has been changed for Lab 5


  private SampleProvider lightMeanL;
  public float[] lightDataL;
  private SampleProvider lightMeanR;
  public float[] lightDataR;
  
  public ColorSensorLocalization(Navigation nav, Odometer odo, SampleProvider lightMean1, float[] lightData1, SampleProvider lightMean2, float[] lightData2) {
      this.nav = nav;
      this.odo = odo;
      this.lightMeanL = lightMean1;
      this.lightDataL = lightData1;
      this.lightMeanR = lightMean2;
      this.lightDataR = lightData2;
  }

  public void run() {
    leftMotor.stop();
    rightMotor.stop();
   
    // Read both sensors once at first
    lightMeanL.fetchSample(lightDataL, 0); // acquire data
	int lightL = (int) (lightDataL[0] * 100.0); // extract from buffer, cast to int
	
	lightMeanR.fetchSample(lightDataR, 0); // acquire data
	int lightR = (int) (lightDataR[0] * 100.0); // extract from buffer, cast to int
	
	// Move forwards until first sensor hits line
	nav.moveForward(30, false, 30);
	while (lightL > COLOR_THRESHOLD && lightR > COLOR_THRESHOLD) { // move forward until you hit a black band
		lightMeanL.fetchSample(lightDataL, 0); // acquire data
		lightL = (int) (lightDataL[0] * 100.0); // extract from buffer, cast to int
		lightMeanR.fetchSample(lightDataR, 0); // acquire data
		lightR = (int) (lightDataR[0] * 100.0); // extract from buffer, cast to int
	}
	Sound.beep();
	nav.stopMotors();
	
	// Move whichever sensor didn't hit line until it hits the line
	if (lightL > COLOR_THRESHOLD) {
		nav.moveLeftMotor(10, false, 30);
		while (lightL > COLOR_THRESHOLD) {
			lightMeanL.fetchSample(lightDataL, 0); // acquire data
			lightL = (int) (lightDataL[0] * 100.0); // extract from buffer, cast to int
		}
		Sound.beep();
		nav.stopMotors();
	} else if (lightR > COLOR_THRESHOLD) {
		nav.moveRightMotor(10, false, 30);
		while (lightR > COLOR_THRESHOLD) {
			lightMeanR.fetchSample(lightDataR, 0); // acquire data
			lightR = (int) (lightDataR[0] * 100.0); // extract from buffer, cast to int
		}
		Sound.beep();
		nav.stopMotors();
	}
	
	Sound.beepSequence();
	odo.setX(0-LIGHT_X_OFFSET); // set Y coordinate to 0
    
	nav.moveBackward(5, true);
	nav.rotate(true, 90, true);
	
	// Read both sensors once at first
    lightMeanL.fetchSample(lightDataL, 0); // acquire data
	lightL = (int) (lightDataL[0] * 100.0); // extract from buffer, cast to int
	
	lightMeanR.fetchSample(lightDataR, 0); // acquire data
	lightR = (int) (lightDataR[0] * 100.0); // extract from buffer, cast to int
	
	// Move forwards until first sensor hits line
	nav.moveForward(30, false, 30);
	while (lightL > COLOR_THRESHOLD && lightR > COLOR_THRESHOLD) { // move forward until you hit a black band
		lightMeanL.fetchSample(lightDataL, 0); // acquire data
		lightL = (int) (lightDataL[0] * 100.0); // extract from buffer, cast to int
		lightMeanR.fetchSample(lightDataR, 0); // acquire data
		lightR = (int) (lightDataR[0] * 100.0); // extract from buffer, cast to int
	}
	Sound.beep();
	nav.stopMotors();
	
	// Move whichever sensor didn't hit line until it hits the line
	if (lightL > COLOR_THRESHOLD) {
		nav.moveLeftMotor(10, false, 30);
		while (lightL > COLOR_THRESHOLD) {
			lightMeanL.fetchSample(lightDataL, 0); // acquire data
			lightL = (int) (lightDataL[0] * 100.0); // extract from buffer, cast to int
		}
		Sound.beep();
		nav.stopMotors();
	} else if (lightR > COLOR_THRESHOLD) {
		nav.moveRightMotor(10, false, 30);
		while (lightR > COLOR_THRESHOLD) {
			lightMeanR.fetchSample(lightDataR, 0); // acquire data
			lightR = (int) (lightDataR[0] * 100.0); // extract from buffer, cast to int
		}
		Sound.beep();
		nav.stopMotors();
	}
	Sound.beepSequence();
	odo.setY(0-LIGHT_Y_OFFSET); // set Y coordinate to 0

    nav.travelTo(0, 0);
    // let robot turn back to 0 degree
    nav.turnTo(0);
  }
  

}