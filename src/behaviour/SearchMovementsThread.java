package behaviour;

import lejos.hardware.Button;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.filter.MedianFilter;
import lejos.robotics.navigation.DifferentialPilot;

public class SearchMovementsThread extends Thread{ 
	
	private DifferentialPilot pilot;
	private int rotationStep = 5;
	private int performedRotation = 0;
	private int fullRotationCounter = 2;
	
	public SearchMovementsThread(DifferentialPilot pilot) {
		this.pilot = pilot;
		
	}
	
	public void run() { 
		while (!this.isInterrupted()) {
			if (fullRotationCounter == 2) {
				fullRotationCounter = 0;
				pilot.forward();
				pilot.travel(4);
			}
			pilot.rotate(rotationStep);
			performedRotation += rotationStep;
			if (Math.abs(performedRotation) == 70) {
				if (rotationStep > 0) {
					pilot.rotate(-70);
				} else {
					pilot.rotate(70);
				}
				rotationStep = rotationStep*(-1);
				fullRotationCounter++;
			}
			if (Button.readButtons() != 0) {
				this.interrupt();
			}
		}
	}
}
