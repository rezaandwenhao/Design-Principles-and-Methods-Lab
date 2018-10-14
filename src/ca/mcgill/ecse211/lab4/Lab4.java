// Lab3.java
package ca.mcgill.ecse211.lab4;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MeanFilter;

/**
 * Main Class for Lab 4 Localizer Lab - ECSE 211 Fall 2018
 * @author Eliott Bourachot
 *
 */
public class Lab4 {

  // Motor Objects, and Robot related parameters
  private static final EV3LargeRegulatedMotor leftMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
  private static final EV3LargeRegulatedMotor rightMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
  private static final Port usPort = LocalEV3.get().getPort("S1");
  private static final Port lightPort = LocalEV3.get().getPort("S2");
  private static final TextLCD lcd = LocalEV3.get().getTextLCD();
  public static final double WHEEL_RAD = 2.13; // (cm) measured with caliper
  public static double TRACK = 9.0; // (cm) measured with caliper
 
  public static void main(String[] args) throws OdometerExceptions {
    
	// Odometer Thread
	Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
	Thread odoThread = new Thread(odometer);
	odoThread.start();

	// Initializing Ultrasonic Sensor and runs it in this thread
	@SuppressWarnings("resource") // Because we don't bother to close this resource
	SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
	SampleProvider usSample = usSensor.getMode("Distance"); 
	SampleProvider usMean = new MeanFilter(usSample, 5); // use a mean filter to reduce fluctuations
    float[] usData = new float[usMean.sampleSize()]; // usData is the buffer in which data are returned
    
    Navigation nav = new Navigation(leftMotor, rightMotor, WHEEL_RAD, WHEEL_RAD, TRACK, odometer);
    
    final UltrasonicLocalizer ul = new UltrasonicLocalizer(usMean, usData, nav, odometer);

    int buttonChoice = 0;

    do {
		lcd.clear(); // clear the display
	  	lcd.drawString("<Left   | Right  >", 0, 0);
	  	lcd.drawString("<Rising | Falling>", 0, 1);
	  	lcd.drawString("<Edge   | Edge   >", 0, 2);
	    
	  	buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
    } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);
    
    final int buttonChoice2 = buttonChoice;

    // Display Thread
    Display odometryDisplay = new Display(lcd); // No need to change
    Thread odoDisplayThread = new Thread(odometryDisplay);
	odoDisplayThread.start();
    
	// Ultrasonic Sensor Thread
  	(new Thread() {
    	public void run() {
    		if (buttonChoice2 == Button.ID_RIGHT) {
    			ul.fallingEdge();
    		} else {
    			ul.risingEdge();
    		}
    	}
    }).start();
  	
    while (Button.waitForAnyPress() != Button.ID_ENTER);
    
    // Initializing Light Sensor and runs it in this thread
  	@SuppressWarnings("resource") // Because we don't bother to close this resource
  	SensorModes lightSensor = new EV3ColorSensor(lightPort); // lightSensor is the instance
  	SampleProvider lightSample = lightSensor.getMode("Red"); // init Red mode
  	SampleProvider lightMean = new MeanFilter(lightSample, 5); // use a mean filter to reduce fluctuations
    float[] lightData = new float[lightMean.sampleSize()]; // usData is the buffer in which data are returned
    
    // Light Localizer Thread
    LightLocalizer ll = new LightLocalizer(nav, odometer, lightMean, lightData);
    Thread llThread = new Thread(ll);
    llThread.run();
    
    while (Button.waitForAnyPress() != Button.ID_ESCAPE);
    System.exit(0);
  }
}
