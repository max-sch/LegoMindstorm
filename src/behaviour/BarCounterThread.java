package behaviour;

import javax.swing.colorchooser.ColorSelectionModel;
import lejos.hardware.Sound;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MedianFilter;
import lejos.utility.Delay;

public class BarCounterThread extends Thread {
	
	private MedianFilter filter;
	private LineBehaviour lineBehaviour;
	boolean firstLow = false;
	boolean isCountingBars = false;
	boolean barCodeComplete = false;
	float realTimeValue;
	long elapsedTime = 0;
	long elapsedTimeSinceLineWasLost = 0;
	long startTime = 0;
	int counter = 0;
	int upperBound = 600;
	int lowerBound = 150;
	
	public BarCounterThread(LineBehaviour line, EV3ColorSensor colorSensor)  {
		this.lineBehaviour = line;
		SampleProvider light = colorSensor.getMode("Red");
		filter = new MedianFilter(light, 10);
	}
	
	@Override
	public void run() {
		while(!this.isInterrupted()) {
			if (!isCountingBars) {
				checkIfBarCodeStarts();
			} else {
				countBarCode();
				checkIfBarCodeIsFinished();
				startBehaviorIfBarCodeComplete();
			}		
		}
	}
	
	private void checkIfBarCodeStarts() {
		realTimeValue = getRealTimeValue();
		if (realTimeValue < 0.1) {
			countElapsedTime();
		} else {
			restartElapsedTimeCounting();
		}
		if (elapsedTime >= 1500) {
			stopFollowingLine();
			startBarCodeCounting();
		}
	}
	
	public void countBarCode() {
		realTimeValue = getRealTimeValue();
		if (realTimeValue < 0.1) {
			countElapsedTime();				
		} else if (firstLow) {
			firstLow = false;
			if (elapsedTime > lowerBound && elapsedTime < upperBound) {
				counter++;
			}
		}
	}
	
	private void checkIfBarCodeIsFinished() {
		if (elapsedTime >= 1500) {
			stopBarCodeCounting();
			barCodeComplete = true;
		}
	}
	
	private void startBehaviorIfBarCodeComplete() {
		if (barCodeComplete) {
			barCodeComplete = false;
			// start Behavior
			counter = 0;
		}
	}
	
	private void restartElapsedTimeCounting() {
		firstLow = false;
	}
	
	private void startBarCodeCounting() {
		elapsedTime = 0;
		isCountingBars = true;
	}
	
	private void stopBarCodeCounting() {
		elapsedTime = 0;
		isCountingBars = false;
	}
	
	private void stopFollowingLine() {
		//todo
	}
	
	
	public void countElapsedTime() {
		if (!firstLow) {
			firstLow = true;
			startTime= System.currentTimeMillis();
		} else {
			elapsedTime = System.currentTimeMillis() - startTime;
		}
	}
	
	public float getRealTimeValue() {
		int samplesize = filter.sampleSize();
		float[] samples = new float[samplesize];
		filter.fetchSample(samples, 0);
		return samples[0];
	}
}