package behaviour;

import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.filter.MedianFilter;

public class LineScanner extends Thread {
	
	private final static float lightThreshold = 0.25f;
	
	private final MedianFilter filter;
	
	public LineScanner(EV3ColorSensor colorSensor) {
		this.filter = new MedianFilter(colorSensor.getRedMode(), 5);
	}
	
	@Override
	public void run() {
		float currentColor = adjustValue(getColor());
		
		while (currentColor < 0.25f) {
			currentColor = adjustValue(getColor());
		}
		
		IBehaviour.setLineFound(true);
	}
	
	private float adjustValue(float value) {
		return value * 1.25f;
	}
	
	private float getColor() {
		int sampleSize = this.filter.sampleSize();
		float[] samples = new float[sampleSize];
		this.filter.fetchSample(samples, 0);
		return samples[0];
	}
}
