package robot;

import java.util.HashMap;

import behaviour.BarCode;
import behaviour.BarCodeScanner;
import behaviour.BridgeBehaviour;
import behaviour.IBehaviour;
import behaviour.LabyrinthBehaviour;
import behaviour.LineBehaviour;
import behaviour.LineScanner;
import behaviour.RollingFieldBehaviour;
import behaviour.RopeBridgeBehaviour;

public class Robot implements IRobot {
	
	private final RobotConfiguration robotConfig;
	private final  HashMap<BarCode, IBehaviour> behaviourWithID;
	private final BarCodeScanner codeScanner;
	
	public Robot(RobotConfiguration robotConfig) {
		this.robotConfig = robotConfig;
		this.behaviourWithID = new HashMap<>();
		this.codeScanner = new BarCodeScanner(robotConfig);
		initBehaviour();
	}
	
	private void initBehaviour() {
		// TODO has to be completed
		this.behaviourWithID.put(BarCode.LABYRINTH, new LabyrinthBehaviour(robotConfig));
		this.behaviourWithID.put(BarCode.LINE, new LineBehaviour(robotConfig));
		this.behaviourWithID.put(BarCode.BRIDGE, new BridgeBehaviour(robotConfig));
		this.behaviourWithID.put(BarCode.ROPEBRIDGE, new RopeBridgeBehaviour(robotConfig));
		this.behaviourWithID.put(BarCode.ROLLINGFIELD, new RollingFieldBehaviour(robotConfig));
	}

	@Override
	public void passParkour() {
		BarCode code = this.codeScanner.scan();
		LineScanner scanner = null;
//		boolean bool = true;
		
		while (!code.equals(BarCode.FINISH)) {
			
			if (!code.equals(BarCode.LINE)) {
				scanner = new LineScanner(robotConfig.getColorSensor());
				scanner.start();
			}
			
//			if(bool  && code.equals(BarCode.LINE)) {
//				code = BarCode.BRIDGE;
//				bool = false;
//			}
			
			this.behaviourWithID.get(code).passObstacle();
			code = this.codeScanner.scan();
			
			if (scanner != null) {
				if (scanner.isAlive()) {
					scanner.interrupt();
				}
				scanner = null;
			}
		}
		
		this.behaviourWithID.get(BarCode.LABYRINTH).passObstacle();
	}

	@Override
	public void startRace() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void passObstacleWithBarCode(BarCode code) {
		LineScanner scanner = null;
		
		if (!code.equals(BarCode.LINE)) {
			scanner = new LineScanner(robotConfig.getColorSensor());
			scanner.start();
		}
		
		this.behaviourWithID.get(code).passObstacle();
		
		if (scanner != null) {
			if (scanner.isAlive()) {
				scanner.interrupt();
			}
		}
		
		IBehaviour.setLineFound(false);
	}
}
