package main;

import lejos.hardware.Button;
import robot.Robot;

public class Main {

	public static void main(String[] args) {
		Robot robot = new Robot();
		robot.initializeMotors();
		//Actions actions = robot.createActions();
		Button.LEDPattern(6);
		Button.waitForAnyPress();
		//Line lineBehavior = robot.createNewLineBehavior();
		//lineBehavior.follow();
		//actions.driveForward();
		//robot.test();
		//Button.waitForAnyPress();
		//robot.test2();
		//actions.stopMotors();
		robot.testUltraSonic();
		
		
	}

}
