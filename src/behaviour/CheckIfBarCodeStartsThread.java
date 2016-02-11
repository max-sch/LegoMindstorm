package behaviour;

import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.filter.MedianFilter;

public class CheckIfBarCodeStartsThread extends Thread{
	private final MedianFilter filter;
	
	public CheckIfBarCodeStartsThread(EV3ColorSensor colorSensor) {
		this.filter = new MedianFilter(colorSensor.getRedMode(), 5);
	}
	
	@Override
	public void run() {
		float firstTime = System.currentTimeMillis();
		//float secondTime
	}
	
	private float getColor() {
		int sampleSize = this.filter.sampleSize();
		float[] samples = new float[sampleSize];
		this.filter.fetchSample(samples, 0);
		return samples[0];
	}
	
}
