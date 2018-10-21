package ca.mcgill.ecse211.lab5;

import ca.mcgill.ecse211.odometer.Odometer;

public class Search extends Thread{

	private static final double TILE_SIZE = 30.48;
	private Navigation nav;
	private Odometer odo;
	private int LLx, LLy, URx, URy, TR, SC;
	
	public Search(Navigation nav, Odometer odo, int[] settings) {
		this.nav = nav;
		this.odo = odo;
		this.LLx = settings[0]; this.LLy = settings[1]; this.URx = settings[2];
		this.URy = settings[3]; this.TR = settings[4]; this.SC = settings[5];
	}
	
	public void run() {
		nav.travelTo(LLx*TILE_SIZE, LLy*TILE_SIZE, false);
		
		// start the search
		
		// get the coordinates of points we want to navigate to
		// these will be the middle of the squares
		
		// move to middle of square
		
		// turn color sensor to left
		
		// while moving to each of the previously calculated coordinates
			// search for ring
			// if a ring is detected
				// figure out its color
				// if its the TC
					// beep once
				// else
					// beep twice
		
		// done with all the coordinates
		// navigate to URx, URy
	}
}

