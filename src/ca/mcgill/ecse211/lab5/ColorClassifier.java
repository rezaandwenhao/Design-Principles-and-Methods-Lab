package ca.mcgill.ecse211.lab5;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.Color;
import lejos.robotics.SampleProvider;

public class ColorClassifier extends Thread {

	private static int COLOR_CORRECTION_PERIOD = 100;
	private static ColorClassifier cc;
	private SampleProvider colorMean;
	private float[] colorData;
	private String detectedColor;
	
	public ColorClassifier(SampleProvider colorMean, float[] colorData) {
		this.colorMean = colorMean;
		this.colorData = colorData;
		this.detectedColor = null;
	}
	
	/**
	   * This class is meant to return the existing Color Classifier Object. It is meant to be used only if an
	   * Color Classifier object has been created
	   * @return error if no previous Color Classifier exists
	   */
	public synchronized static ColorClassifier getColorClassifier() throws OdometerExceptions {
	
	  if (cc == null) {
	    throw new OdometerExceptions("No previous Color Classifier exits.");
	  }
	  return cc;
	}
	
	public synchronized static ColorClassifier getColorClassifier(SampleProvider colorMean, float[] colorData)
	      throws OdometerExceptions {
	    if (cc != null) { // Return existing object
	      return cc;
	    } else { // create object and return it
	      cc = new ColorClassifier(colorMean, colorData);
	      return cc;
	    }
	  }	
	 
	public void run() {
	    long updateStart, updateEnd;

		while(true) {
		    updateStart = System.currentTimeMillis();

			colorMean.fetchSample(colorData, 0);
		    float red = colorData[0], green = colorData[1], blue = colorData[2];
		    if (red > green && red > blue) {
		    	detectedColor = "Red";
		    } else if (green > red && green > blue) {
		    	detectedColor = "Green";
		    } else if (blue > red && blue > green) {
		    	detectedColor = "Blue";
		    } else {
		    	detectedColor = "None";
		    }
		    
		    // this ensures that the odometer only runs once every period
		      updateEnd = System.currentTimeMillis();
		      if (updateEnd - updateStart < COLOR_CORRECTION_PERIOD) {
		        try {
		          Thread.sleep(COLOR_CORRECTION_PERIOD - (updateEnd - updateStart));
		        } catch (InterruptedException e) {
		          // there is nothing to be done
		        }
		      }
		}
	}	   
	
	public float[] getColorData() {
		return colorData;
	}
	
	public String getDetectedColor() {
		return detectedColor;
	}
}
