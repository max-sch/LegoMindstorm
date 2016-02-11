package behaviour;

public abstract class IBehaviour {
	protected static boolean lineFound = false;
	
	public static void setLineFound(boolean value) {
		lineFound = value;
	}
	
	public abstract BarCode passObstacle();
}
