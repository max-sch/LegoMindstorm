package test;

import behaviour.ScanForLineThread;
import behaviour.SearchMovementsThread;
import lejos.hardware.Button;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.navigation.DifferentialPilot;
import robot.Robot;
import robot.RobotConfiguration;

public class Line3 {
	private RegulatedMotor rightMotor;
	private RegulatedMotor leftMotor;
	private EV3ColorSensor colorSensor;
	private float normalLight = 0.2f;
	private boolean onLine;
	private boolean direction = true;
	private float realTimeCValue = 0;
	private float oldRealTimeValue;
	DifferentialPilot pilot;
	
	public Line3(RobotConfiguration robotConfig) {
		this.rightMotor = robotConfig.getRightMotor();
		this.leftMotor = robotConfig.getLeftMotor();
		this.colorSensor = robotConfig.getColorSensor();
		pilot = new DifferentialPilot(2,10, leftMotor, rightMotor);
	}
	
	public void searchLine() {
		SearchMovementsThread movementThread = new SearchMovementsThread(pilot);
		movementThread.start();
		ScanForLineThread scanThread = new ScanForLineThread(this, colorSensor, movementThread);
		scanThread.start();		
	}
	
	public void setLineStartingPosition() {
		boolean goodStartingPosition = false;
		int deg = -2;
		oldRealTimeValue = getRealTimeValue(); 
		while (!goodStartingPosition) {
			pilot.rotate(deg);
			if (getRealTimeValue() < oldRealTimeValue) {
				if (getRealTimeValue() > 0.15) {
					goodStartingPosition = true;
				} else {
					pilot.rotate(-deg);
					goodStartingPosition = true;
				}
			} 
			oldRealTimeValue = getRealTimeValue(); 
			if (Button.readButtons() != 0) {
				break;
			}
		}
	}
	
	private boolean isOnLine() {
		return (getRealTimeValue() > normalLight);
	}
	
	private float getRealTimeValue() {
		int samplesize = colorSensor.sampleSize();
		float[] samples = new float[samplesize];
		colorSensor.getRedMode().fetchSample(samples, 0);
		return samples[0];
	}
}
