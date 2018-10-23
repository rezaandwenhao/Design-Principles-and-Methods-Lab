// Lab5.java
package ca.mcgill.ecse211.lab5;

import ca.mcgill.ecse211.lab5.ColorClassifier.Color;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MeanFilter;

/**
 * Main Class for Lab 5 - ECSE 211 Fall 2018
 * 
 * @author Eliott Bourachot
 */
public class Lab5 {

  // Motor Objects
  static final EV3LargeRegulatedMotor leftMotor =
		  new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
  static final EV3LargeRegulatedMotor rightMotor =
		  new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
  static final EV3MediumRegulatedMotor mediumMotor =
		  new EV3MediumRegulatedMotor(LocalEV3.get().getPort("B"));
  // Sensors
  private static final Port usPort = LocalEV3.get().getPort("S1");
  private static final Port lightLPort = LocalEV3.get().getPort("S4");
  private static final Port lightRPort = LocalEV3.get().getPort("S2");
  private static final Port colorPort = LocalEV3.get().getPort("S3"); 
  
  // Robot related parameters
  private static final TextLCD lcd = LocalEV3.get().getTextLCD();
  public static final double WHEEL_RAD = 2.13; // (cm) measured with caliper
  public static double TRACK = 9.0; // (cm) measured with caliper
 
  public static void main(String[] args) throws OdometerExceptions {
    
	// Odometer Thread
	Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
	Thread odoThread = new Thread(odometer);
	odoThread.start();
	
    // Initializing Light Sensor 1 and runs it in this thread
    @SuppressWarnings("resource") // Because we don't bother to close this resource
    SensorModes lightSensor1 = new EV3ColorSensor(lightLPort); // lightSensor is the instance
    SampleProvider lightSample1 = lightSensor1.getMode("Red"); // init Red mode
    SampleProvider lightMean1 = new MeanFilter(lightSample1, 5); // use a mean filter to reduce fluctuations
    float[] lightData1 = new float[lightMean1.sampleSize()]; // usData is the buffer in which data are returned
    
    // Initializing Light Sensor 2 and runs it in this thread
    @SuppressWarnings("resource") // Because we don't bother to close this resource
    SensorModes lightSensor2 = new EV3ColorSensor(lightRPort); // lightSensor is the instance
    SampleProvider lightSample2 = lightSensor2.getMode("Red"); // init Red mode
    SampleProvider lightMean2 = new MeanFilter(lightSample2, 5); // use a mean filter to reduce fluctuations
    float[] lightData2 = new float[lightMean2.sampleSize()]; // usData is the buffer in which data are returned
    
    // Navigation
    Navigation nav = new Navigation(leftMotor, rightMotor, WHEEL_RAD, WHEEL_RAD, TRACK, odometer, mediumMotor,
        lightMean1, lightData1, lightMean2, lightData2);

    // Display Thread
    Display generalDisplay = new Display(lcd); // No need to change
    Thread displayThread = new Thread(generalDisplay);
    displayThread.start();

	// Initializing Ultrasonic Sensor and runs it in this thread
	@SuppressWarnings("resource") // Because we don't bother to close this resource
	SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
	SampleProvider usSample = usSensor.getMode("Distance"); 
	//SampleProvider usMean = new MeanFilter(usSample, 5); // use a mean filter to reduce fluctuations
    float[] usData = new float[usSample.sampleSize()]; // usData is the buffer in which data are returned
 
	UltrasonicLocalizer ul = new UltrasonicLocalizer(usSample, usData, nav, odometer, true);
	ul.start();
      
    while (Button.waitForAnyPress() != Button.ID_ENTER);

    // Light Localizer Thread
    ColorSensorLocalization csl = new ColorSensorLocalization(nav, odometer, lightMean1, lightData1, lightMean2, lightData2);
    csl.start();
    
    while (Button.waitForAnyPress() != Button.ID_ENTER);

    // Initializing Color Sensor and runs it in this thread
 	@SuppressWarnings("resource") // Because we don't bother to close this resource
 	SensorModes colorSensor = new EV3ColorSensor(colorPort); // lightSensor is the instance
 	SampleProvider colorSample = colorSensor.getMode("ColorID"); // init RGB mode
 	SampleProvider colorMean = new MeanFilter(colorSample, 5); // use a mean filter to reduce fluctuations
 	float[] colorData = new float[colorSensor.sampleSize()]; // usData is the buffer in which data are returned
   
 	// Color Classifier Thread
 	ColorClassifier cc = new ColorClassifier(colorMean, colorData, Color.orange);
    cc.start();
    
    // Navigation Thread
 	nav.start();

    
    while (Button.waitForAnyPress() != Button.ID_ESCAPE);
    System.exit(0);
  }
}
