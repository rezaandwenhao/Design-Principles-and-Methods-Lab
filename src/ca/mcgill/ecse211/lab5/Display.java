package ca.mcgill.ecse211.lab5;

import java.text.DecimalFormat;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.lcd.TextLCD;
import lejos.robotics.Color;

/**
 * This class is used to display the content of the odometer variables (x, y, Theta)
 */
public class Display implements Runnable {

  private Odometer odo;
  private ColorClassifier cc;
  private TextLCD lcd;
  private float[] colorData;
  private double[] position;
  private final long DISPLAY_PERIOD = 25;
  private long timeout = Long.MAX_VALUE;

  /**
   * This is the class constructor
   * 
   * @param odoData
   * @throws OdometerExceptions 
   */
  public Display(TextLCD lcd) throws OdometerExceptions {
    odo = Odometer.getOdometer();
    cc = ColorClassifier.getColorClassifier();
    this.lcd = lcd;
  }

  /**
   * This is the overloaded class constructor
   * 
   * @param odoData
   * @throws OdometerExceptions 
   */
  public Display(TextLCD lcd, long timeout) throws OdometerExceptions {
    odo = Odometer.getOdometer();
    cc = ColorClassifier.getColorClassifier();
    this.timeout = timeout;
    this.lcd = lcd;
  }

  public void run() {
    
    lcd.clear();
    
    long updateStart, updateEnd;

    long tStart = System.currentTimeMillis();
    do {
      updateStart = System.currentTimeMillis();

      // Retrieve x, y and Theta information
      position = odo.getXYT();
      // Retrieve red, green, blue information
      colorData = cc.getColorData();
      
      // Print x,y, and theta information
      DecimalFormat numberFormat = new DecimalFormat("######0.00");
      lcd.drawString("X: " + numberFormat.format(position[0]), 0, 0);
      lcd.drawString("Y: " + numberFormat.format(position[1]), 0, 1);
      lcd.drawString("T: " + numberFormat.format(position[2]), 0, 2);
      lcd.drawString("Red: " + colorData[0], 0, 3);
	  lcd.drawString("Green: " + colorData[1], 0, 4);
	  lcd.drawString("Blue: " + colorData[2], 0, 5);
	  lcd.drawString("Detected: " + cc.getDetectedColor(), 0, 5);

      // this ensures that the data is updated only once every period
      updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < DISPLAY_PERIOD) {
        try {
          Thread.sleep(DISPLAY_PERIOD - (updateEnd - updateStart));
        } catch (InterruptedException e) {
          e.printStackTrace();
        }
      }
    } while ((updateEnd - tStart) <= timeout);

  }

}
