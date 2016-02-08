package behaviour;

import robot.RobotConfiguration;

public class LineBehaviourThread extends Thread {
	
	private final LineBehaviour line;
	
	private boolean isStopped;
	
	public LineBehaviourThread(RobotConfiguration robotConfigs) {
		this.line = new LineBehaviour(robotConfigs);
		isStopped = false;
	}
	
	@Override
	public void run() {
		while (!isStopped) {
			this.line.passObstacle();
		}
	}

	public boolean isStopped() {
		return isStopped;
	}

	public void setStopped(boolean isStopped) {
		this.isStopped = isStopped;
	}
}
