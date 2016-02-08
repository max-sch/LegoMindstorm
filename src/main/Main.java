package main;

import behaviour.BarCode;
import lejos.hardware.Button;
import robot.IRobot;
import robot.Robot;
import robot.RobotConfiguration;

public class Main {

	public static void main(String[] args) {
		RobotConfiguration robotConfig = new RobotConfiguration();
		robotConfig.initializeMotors();
		
		IRobot robot = new Robot(robotConfig);
		
		Button.LEDPattern(6);
		Button.waitForAnyPress();
		
		robot.passObstacleWithBarCode(BarCode.LABYRINTH);
	}

}
