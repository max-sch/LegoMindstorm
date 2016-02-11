package main;

import org.jfree.data.statistics.DefaultBoxAndWhiskerXYDataset;

import behaviour.BarCode;
import behaviour.BarCodeScanner;
import behaviour.LineBehaviour;
import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.utility.Delay;
import robot.Robot;
import robot.RobotConfiguration;
import test.SensorTest;

public class Main {

	public static void main(String[] args) {
		RobotConfiguration robotConfig = new RobotConfiguration();
		robotConfig.initializeMotors();
		
		Button.LEDPattern(6);
		Button.waitForAnyPress();
		
		Robot robot = new Robot(robotConfig);
		robot.passParkour();
		//LineBehaviour b = new LineBehaviour(robotConfig);
		//b.findLine();
		//robot.passObstacleWithBarCode(BarCode.LABYRINTH);
		//BarCodeScanner scanner = new BarCodeScanner(robotConfig);
		//BarCode code = scanner.scan();
//		DifferentialPilot pilot = new DifferentialPilot(2, 10, robotConfig.getLeftMotor(), robotConfig.getRightMotor());
//		pilot.forward();
//		
//		scanner.isDone();
//		
//		pilot.stop();
		
		//LCD.drawString("Val: " + code.toString(), 0, 1);
		
		
		Button.LEDPattern(6);
		Button.waitForAnyPress();
		//SensorTest.testColorID(robotConfig.getColorSensor());
//		LabyrinthBehaviour be = new LabyrinthBehaviour(robotConfig);
//		be.passObstacle();
		//BridgeBehaviour br = new BridgeBehaviour(robotConfig);
		//br.passObstacle();
		//robot.passObstacleWithBarCode(BarCode.LABYRINTH);
	}

}
