package behaviour;

import robot.RobotConfiguration;

public class LabyrinthBehaviourThread extends Thread {
	private final LabyrinthBehaviour laby;
	
	public LabyrinthBehaviourThread(RobotConfiguration robotConfigs) {
		this.laby = new LabyrinthBehaviour(robotConfigs);
	}
	
	@ Override
	public void run() {
		this.laby.passObstacle();
	}
}