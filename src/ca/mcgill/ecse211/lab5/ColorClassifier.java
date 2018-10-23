package ca.mcgill.ecse211.lab5;

import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;

/**
 * This class is used to detect the color using light sensor ColorID mode
 * It is always running once the thread is launched
 * The thread let the robot beep once if a desired color ring is detected, 
 * and beep twice if other color ring is detected. It does not beep when detect nothing
 * 
 * @author Eliott Bourachot
 * @edits Wenhao Geng, Eden Ovadia
 */

public class ColorClassifier extends Thread {

	public static Color detectedColor;
	public static enum Color {blue, orange, yellow, green, nothing};

	private static int COLOR_CORRECTION_PERIOD = 10;
	private SampleProvider colorSampleP;
	private float[] sampleColor;
	private Color SC;
	private static int FILTER = 3;
	
	public ColorClassifier(SampleProvider colorMean, float[] colorData, Color SC) {
		ColorClassifier.detectedColor = Color.nothing;
		this.colorSampleP = colorMean;
		this.sampleColor = colorData;
		this.SC = SC;
		
	}
	public void run() {
	    long updateStart, updateEnd;
	    int numTimesDetected = 0;
	    Color pastDetectedColor = Color.nothing;
	    boolean detectionEnabled = false;
	    long detectedTime = 0;
	    
		while(true) {
		    updateStart = System.currentTimeMillis();

			colorSampleP.fetchSample(sampleColor, 0);

			// classifies color
			if (1.5 < sampleColor[0] && sampleColor[0] < 2.5) {
			  detectedColor = Color.blue;
			} else if (5.5 < sampleColor[0] && sampleColor[0] < 6.5) {
			  detectedColor = Color.green;
			} else if (2.5 < sampleColor[0] && sampleColor[0] < 3.5) {
			  detectedColor = Color.yellow;
			} else if (12.5 < sampleColor[0] && sampleColor[0] < 13.5) {
              detectedColor = Color.orange;
			} else {
			  detectedColor = Color.nothing;
			}
			
			// beeps once if detects correct color (more than FILTER times)
			if (detectedColor == pastDetectedColor && detectedColor != Color.nothing && !detectionEnabled) {
				if (numTimesDetected > FILTER) {
					if (detectedColor == SC) {
						Sound.beep();
						detectionEnabled = true;
						detectedTime = System.currentTimeMillis();
					} else {
						Sound.twoBeeps();
						detectionEnabled = true;
						detectedTime = System.currentTimeMillis();
					}
				} else 
					numTimesDetected ++;
			} else {
				numTimesDetected = 0;
			}
			
			// need to wait seconds to reset detection 
			if (System.currentTimeMillis() - detectedTime > 2000) {
				detectionEnabled = false;
			}
			
			pastDetectedColor = detectedColor;
		
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
	
	public Color getDetectedColor() {
		return detectedColor;
	}
}
