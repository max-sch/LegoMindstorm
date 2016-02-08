package behaviour;

import javax.swing.colorchooser.ColorSelectionModel;

import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MedianFilter;

public class BarCounterThread extends Thread {
	
	private EV3ColorSensor colorSensor;
	private MedianFilter filter;
	private LineBehaviour lineBehaviour;
	
	public BarCounterThread(LineBehaviour line, EV3ColorSensor colorSensor)  {
		this.lineBehaviour = line;
		this.colorSensor = colorSensor;
		SampleProvider light = colorSensor.getMode("Red");
		filter = new MedianFilter(light, 10);
	}
	
	@Override
	public void run() {
		int counter = 0;
		boolean firstHigh = false;
		float previousRealTimeValue;
		float realTimeValue;
		long elapsedTime = 0;
		long startTime = 0;
		while(!this.isInterrupted()) {
			realTimeValue = getRealTimeValue();
			if (realTimeValue > 0.25) {
				if (!firstHigh) {
					firstHigh = true;
					startTime = System.currentTimeMillis();
				} else {
					elapsedTime = System.currentTimeMillis() - startTime;
				}				
			} else if (firstHigh) {
				firstHigh = false;
				if (elapsedTime > 200) {
					lineBehaviour.barFound();
				}
			}
			
		}
	}
	
	public float getRealTimeValue() {
		int samplesize = filter.sampleSize();
		float[] samples = new float[samplesize];
		filter.fetchSample(samples, 0);
		return samples[0];
	}
}