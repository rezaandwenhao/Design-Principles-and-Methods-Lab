package ca.mcgill.ecse211.lab5;

import ca.mcgill.ecse211.odometer.Odometer;

public class Search extends Thread{

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
		nav.travelTo(LLx, LLy);
	}
}

