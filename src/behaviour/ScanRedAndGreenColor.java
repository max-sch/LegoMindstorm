package behaviour;

import lejos.hardware.lcd.LCD;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MedianFilter;

public class ScanRedAndGreenColor {

	private final EV3ColorSensor colorSensor;
	//private final BridgeBehaviour bridge;
	
	private final SampleProvider provider;
	private final MedianFilter filter;
	
	public ScanRedAndGreenColor(EV3ColorSensor colorSensor) {
		this.colorSensor = colorSensor;
		//this.bridge = bridge;
		
		provider = this.colorSensor.getAmbientMode();
		filter = new MedianFilter(provider, 10);
	}
	
//	@Override
//	public void run() {
//		while(!this.isInterrupted()) {
//			
//			if (getColor() < 0.05f) {
//				bridge.wasFound = true;
//				this.interrupt();
//			}
//		}
//		
//	}
	
	public float getColor() {
		int sampleSize = filter.sampleSize();
		float[] samples = new float[sampleSize];
		filter.fetchSample(samples, 0);
		
		return samples[0];
	}
}
