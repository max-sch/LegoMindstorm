package test;

import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;

public class SensorTest {

	public static void testColorSensor(EV3ColorSensor colorSensor) {
		//colorSensor.setFloodlight();
		while (true) {
			//MedianFilter filter = new MedianFilter(colorSensor.getRedMode(), 10);
			int samplesize = colorSensor.sampleSize();
			float[] samples = new float[samplesize];
			colorSensor.getRedMode().fetchSample(samples, 0);
			
	    	LCD.drawString("ID =" + samples[0],0,1); 
	    	
	    	if (Button.readButtons() != 0) {
	    		break;
	    	}
		}
	}
	
	public static void testTouchSensors(EV3TouchSensor touchSensor) {
		while (true) {
			int samplesize = touchSensor.sampleSize();
			float[] samples = new float[samplesize];
			touchSensor.fetchSample(samples, 0);
			
			LCD.drawString("ID =" + samples[0],0,1); 
	    	
	    	if (Button.readButtons() != 0) {
	    		break;
	    	}
		}
	}
	
	public static void testUltraSonic(EV3UltrasonicSensor ultraSonicSensor) {
		while (true) {
			int samplesize = ultraSonicSensor.sampleSize();
			float[] samples = new float[samplesize];
			ultraSonicSensor.fetchSample(samples, 0);
		
	    	LCD.drawString("Val =" + samples[0],0,1); 
	    	
	    	if (Button.readButtons() != 0) {
	    		break;
	    	}
		}
	}
}
