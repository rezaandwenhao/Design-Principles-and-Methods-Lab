package ca.mcgill.ecse211.lab5;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.Color;
import lejos.robotics.SampleProvider;

public class ColorClassifier extends Thread {

	private static int COLOR_CORRECTION_PERIOD = 100;
	private static ColorClassifier cc;
//	private SampleProvider colorProvider;
//	private float[] colorData;
	public static String detectedColor;
	//public static float red, green, blue;
	
	  // initializing color sensor, color sample provider, and an array to hold samples
	  private static Port colorSensor = LocalEV3.get().getPort("S3");
	  private static SensorModes mode = new EV3ColorSensor(colorSensor);
	  private static SampleProvider colorSampleP = mode.getMode("ColorID");
	  private static float[] sampleColor = new float[mode.sampleSize()];
	
	public ColorClassifier(SampleProvider colorMean, float[] colorData) {
//		this.colorProvider = colorProvider;
//		this.colorData = colorData;
		ColorClassifier.detectedColor = null;
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

			colorSampleP.fetchSample(sampleColor, 0);
			
		   // red = colorData[0];
		   // System.out.println("red: " + red*1000);
		   // green = colorData[1]; 
		   // blue = colorData[2];
			if (1.5 < sampleColor[0] && sampleColor[0] < 2.5) {
			  detectedColor= "blue";
			} else if (5.5 < sampleColor[0] && sampleColor[0] < 6.5) {
			  detectedColor= "green";
			} else if (2.5 < sampleColor[0] && sampleColor[0] < 3.5) {
			  detectedColor= "yellow";
			} else if (-0.5 < sampleColor[0] && sampleColor[0] < 0.5) {
              detectedColor= "orange";
			} else {
			  detectedColor="Nothing";
			}
		    /*if (red > green && red > blue) {
		    	detectedColor = "Red";
		    } else if (green > red && green > blue) {
		    	detectedColor = "Green";
		    } else if (blue > red && blue > green) {
		    	detectedColor = "Blue";
		    } else {
		    	detectedColor = "None";
		    }*/
		    
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
		return sampleColor;
	}
	
	public String getDetectedColor() {
		return detectedColor;
	}
}
