package behaviour;

import java.util.HashMap;

import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.filter.MedianFilter;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.utility.Delay;
import robot.RobotConfiguration;

public class BarCodeScanner {

	private final RegulatedMotor leftMotor;
	private final RegulatedMotor rightMotor;
	private final MedianFilter filter;
	
	private int counter = 0;
	private HashMap<Integer, BarCode> getBarcode;
	
	public BarCodeScanner(RobotConfiguration robotConfig) {
		this.leftMotor = robotConfig.getLeftMotor();
		this.rightMotor = robotConfig.getRightMotor();
		filter = new MedianFilter(robotConfig.getColorSensor().getRedMode(), 5);
		initBarCodeMap();
	}
	
	private void initBarCodeMap() {
		this.getBarcode = new HashMap<>();
		this.getBarcode.put(0, BarCode.LABYRINTH);
		this.getBarcode.put(2, BarCode.LINE);
		this.getBarcode.put(4, BarCode.LINE);
		this.getBarcode.put(1, BarCode.ROLLINGFIELD);
		this.getBarcode.put(5, BarCode.ROPEBRIDGE);
		this.getBarcode.put(3, BarCode.BRIDGE);
		this.getBarcode.put(6, BarCode.FINISH);
	}
	
	public BarCode scan() {
		float color;
		
		this.leftMotor.setSpeed(50);
		this.rightMotor.setSpeed(50);
		
		DifferentialPilot pilot = new DifferentialPilot(2, 10, leftMotor, rightMotor);
		
		pilot.backward();
		Delay.msDelay(300);
		pilot.stop();
		pilot.forward();
		
		while(true) {
			color = adjustValue(getColor());
			
			if (color > 0.25) {
				this.counter++;
				waitUntilLineIsPassed();
			}
			
			if (isDone()) {
				break;
			}
			
			if (Button.readButtons() != 0) {
				break;
			}
		}
		
		pilot.stop();
		
		BarCode code = getBarcode.get(this.counter); 
		//this.counter = 0;
		
		return code;
	}
	
	private void waitUntilLineIsPassed() {
		float color = adjustValue(getColor());
		
		while (color >= 0.2f) {
			color = adjustValue(getColor());
		}
	}
	
	public boolean isDone() {
		long firstTime = System.currentTimeMillis();
		long secondTime = firstTime;
		long difference;
		float color = adjustValue(getColor());
		
		difference = secondTime - firstTime;
		
		while (difference < 500) {
			 if (color > 0.25f) {
				 return false;
			 }
			 
			 LCD.drawString("dif:" + difference, 0, 1);
			 
			 secondTime = System.currentTimeMillis();
			 
			 color = adjustValue(getColor());
			 
			 if (Button.readButtons() != 0) {
					break;
			}
			 
			difference = secondTime - firstTime;
			
		}
		
		return true;
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
	
	public int getCounter() {
		return this.counter;
	}
}
