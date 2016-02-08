package behaviour;

import lejos.hardware.Button;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MedianFilter;
import test.Line3;

public class ScanForLineThread extends Thread {
	
	public boolean lineFound = false;
	private EV3ColorSensor colorSensor;
	private float normalLight = 0.2f;
	private float oldRealTimeValue;
	private Thread movementThread;
	private Line3 line;
	MedianFilter filter;

	public ScanForLineThread (Line3 line, EV3ColorSensor colorSensor, Thread movementThread) {
		this.line = line;
		this.colorSensor = colorSensor;
		this.movementThread = movementThread;		
		SampleProvider light = colorSensor.getMode("Red");
		filter = new MedianFilter(light, 10);
	}
	
	@Override
	public void run() {
		while (!this.isInterrupted()) {
			stopThreadWhenLineisFound();
			if (Button.readButtons() != 0) {
				movementThread.interrupt();
				this.interrupt();
			}
		}
	}
	
	public void stopThreadWhenLineisFound() {
		if (isOnLine()) {
			movementThread.interrupt();
			line.setLineStartingPosition();
			this.interrupt();
		}
	}
	
	private boolean isOnLine() {
		return (getRealTimeValue() > normalLight);
	}
	
	public float getRealTimeValue() {
		int samplesize = filter.sampleSize();
		float[] samples = new float[samplesize];
		filter.fetchSample(samples, 0);
		return samples[0];
	}
}
