package test;

import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.Color;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MedianFilter;
import lejos.utility.Delay;

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
		SampleProvider provider = ultraSonicSensor.getDistanceMode();
		MedianFilter filter = new MedianFilter(provider, 10);
		while (true) {
			int samplesize = filter.sampleSize();
			float[] samples = new float[samplesize];
			filter.fetchSample(samples, 0);
		
	    	LCD.drawString("Val =" + samples[0],0,1); 
	    	
	    	if (Button.readButtons() != 0) {
	    		break;
	    	}
		}
	}
	
	public static void testColorID(EV3ColorSensor colorSensor) {
		SampleProvider provider = colorSensor.getAmbientMode();
		MedianFilter filter = new MedianFilter(provider, 10);
		
		while (true) {
			
			int samplesize = filter.sampleSize();
			float[] samples = new float[samplesize];
			filter.fetchSample(samples, 0);
						
			LCD.drawString("ID: " + samples[0], 0, 1);
			
			if (Button.readButtons() != 0) {
	    		break;
	    	}
		}
	}
}
