package ca.mcgill.ecse211.lab5;

import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;

public class UltrasonicLocalizer extends Thread{
	
	private final int UPPER_CORRECTION = 225; // Correction for when angle 1 is smaller than angle 2 (degrees)
	private final int LOWER_CORRECTION = 45; // Correction for when angle 1 is larger than angle 2 (degrees)
	private final int VOID_LIMIT = 30; // Distance limit under which we detect a wall (cm)
	private final int VOID_BAND = 3; // Distance error band (cm) 
	private SampleProvider usMean;
	private float[] usData;
	private Navigation nav;
	private Odometer odo;
	private boolean pastView;
	
	public UltrasonicLocalizer(SampleProvider usMean, float[] usData, Navigation nav, Odometer odo) {
		this.usMean = usMean;
		this.usData = usData;
		this.nav = nav;
		this.odo = odo;
		this.pastView = true;
	}
	
	/**
	 * runs a falling edge localization procedure that follows these instructions:
	 * 
	 * Start -> If looking at wall, turn clockwise towards void and a bit more
	 * -> Rotate clockwise until you see a falling edge -> Record angle
	 * Turn counter clockwise until you see a falling edge -> Record angle
	 * Set new angle
	 */
	public void fallingEdge() {
		
		if (seeingSomething()) {
			nav.rotate(true, 360, false); // rotate clockwise
			while (seeingSomething()); // if we're facing the wall, turn to face the void
			nav.stop();
			
			nav.rotate(true, 10, true); // rotate a bit more
			odo.setTheta(odo.getXYT()[2]); // reset 0 angle
		}
		
		nav.rotate(true, 360, false); // rotate clockwise
		while(!seeingSomething());
		nav.stop();
		
		Sound.beep();
		double angle1 = odo.getXYT()[2];

		nav.rotate(false, 360, false); // rotate counter-clockwise
		long snapshot = System.currentTimeMillis();
		while(seeingSomething() || (System.currentTimeMillis() - snapshot > 1000));
		nav.stop();
		
		nav.rotate(false, 360, false); // rotate counter-clockwise
		while(!seeingSomething());
		nav.stop();
		
		Sound.beep();
		double angle2 = odo.getXYT()[2];

		double deltaTheta = getHeading(angle1, angle2);
		
		odo.setTheta(deltaTheta+odo.getXYT()[2]);
		
		nav.turnTo(0);
	}
	
	/**
	 * runs a falling edge localization procedure that follows these instructions:
	 * 
	 * Start -> If looking at void, turn clockwise to wall and a bit more
	 * Turn counter-clockwise until you find a rising edge -> Record angle
	 * Turn clockwise until you see the wall -> Continue until you see another rising edge
	 * Record angle -> Set new angle
	 */
	public void risingEdge() {
		
		if (!seeingSomething()) {
			nav.rotate(true, 360, false); // rotate clockwise
			while (!seeingSomething()); // if we're looking into the void, turn to face the walls
			nav.stop();
			
			nav.rotate(true, 10, true); // rotate a bit more
			odo.setTheta(odo.getXYT()[2]); // reset 0 angle
		}
	
		nav.rotate(false, 360, false); // rotate counter-clockwise
		while(seeingSomething());
		nav.stop();
		
		Sound.beep();
		double angle1 = odo.getXYT()[2];
		
		nav.rotate(true, 360, false); // rotate clockwise
		while(!seeingSomething());
		nav.stop();
		
		nav.rotate(true, 360, false); // rotate clockwise
		while (seeingSomething());
		nav.stop();
		
		Sound.beep();
		double angle2 = odo.getXYT()[2];

		double deltaTheta = getHeading(angle1, angle2);
		
		odo.setTheta(deltaTheta+odo.getXYT()[2]);	
		
		nav.turnTo(0);
	}
	
	/**
	 * @return true if the robot is seeing a wall, false if its seeing the void
	 */
	boolean seeingSomething() {
		usMean.fetchSample(usData, 0); // acquire data
		int distance = (int) (usData[0] * 100.0); // extract from buffer, cast to int
		
		if (distance > VOID_LIMIT + VOID_BAND) {
			this.pastView = false;
			return false;
		} else if (distance < VOID_LIMIT - VOID_BAND) {
			this.pastView = true;
			return true;
		} else {
			return pastView;
		}
	}
	
	/**
	 * @return the correction needed to make the heading of the robot true North
	 * @param angle1
	 * @param angle2
	 */
	private double getHeading(double angle1, double angle2) {
		if (angle1 < angle2) {
			return UPPER_CORRECTION-((angle1+angle2)/2);
		} else {
			angle1 %= 360;
			return LOWER_CORRECTION-((angle1+angle2)/2);
		}
	}
	
	
	
}
