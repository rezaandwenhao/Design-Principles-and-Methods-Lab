package ca.mcgill.ecse211.lab5;

import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;

public class LightLocalizer extends Thread{

	private final int LIGHT_THRESHOLD = 17; // Value under which we consider it a black line
	private final int LIGHT_Y_OFFSET = 5; // Y offset to account for the light sensor not being in the middle (cm)
	private final int LIGHT_X_OFFSET = 5; // X offset to account for the light sensor not being in the middle (cm)
	private Navigation nav;
	private Odometer odo;
	private SampleProvider lightMean;
	private float[] lightData;
	
	public LightLocalizer(Navigation nav, Odometer odo, SampleProvider lightMean, float[] lightData) {
		this.nav = nav;
		this.odo = odo;
		this.lightMean = lightMean;
		this.lightData = lightData;
	}
		
	public void run() {

		lightMean.fetchSample(lightData, 0); // acquire data
		int light = (int) (lightData[0] * 100.0); // extract from buffer, cast to int
		
		nav.moveForward(30, false);
		while (light > LIGHT_THRESHOLD) { // move forward until you hit a black band
			lightMean.fetchSample(lightData, 0); // acquire data
			light = (int) (lightData[0] * 100.0); // extract from buffer, cast to int
		}
		nav.stop(); // stop motors
		
		Sound.beep(); // beep
		odo.setY(0-LIGHT_X_OFFSET); // set X coordinate to 0
		
		nav.turnTo(90); // turn to 90 degrees
		
		lightMean.fetchSample(lightData, 0); // acquire data
		light = (int) (lightData[0] * 100.0); // extract from buffer, cast to int
		
		nav.moveForward(30, false);
		while (light > LIGHT_THRESHOLD) { // move forward until you hit a black band
			lightMean.fetchSample(lightData, 0); // acquire data
			light = (int) (lightData[0] * 100.0); // extract from buffer, cast to int
		}
		nav.stop();
		Sound.beep();
		odo.setX(0-LIGHT_Y_OFFSET); // set Y coordinate to 0
		
		nav.travelTo(0, 0); // go to origin
		nav.turnTo(0); // turn to 0 degrees
		
	}
}
