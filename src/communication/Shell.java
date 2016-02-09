package communication;

import java.io.IOException;
import java.util.Scanner;

public class Shell {

	private static ComModule com;

	public static void main(String[] args) {
		com = Communication.getModule();

		try (Scanner scanner = new Scanner(System.in)) {
			boolean quit = false;
			while (!quit) {
				System.out.print("> ");
				if (scanner.hasNext()) {
					quit = evaluate(scanner.nextLine());
				}
			}
		} catch (Exception e) {
			System.err.println(e.getMessage());
		}
	}

	private static boolean evaluate(String input) throws IOException {
		boolean quit = false;

		long start = System.nanoTime();
		boolean response = false;
		switch (input.toUpperCase()) {
		case "UP":
			System.out.println("Starting request...");
			response = com.requestElevator();
			System.out.println("Called elevator");
			System.out.println("Response: " + response);
			break;
		case "DOWN":
			System.out.println("Starting request...");
			response = com.moveElevatorDown();
			System.out.println("Requested elevator to move down");
			System.out.println("Response: " + response);
			break;
		case "STATUS":
			System.out.println("Starting request...");
			response = com.requestStatus();
			System.out.println("Requested elevator status");
			System.out.println("Response: " + response);
			break;
		case "Q":
		case "QUIT":
		case "EXIT":
			System.out.println("Quitting...");
			quit = true;
			break;
		default:
			System.out.println("Unknown command: " + input);
		}

		long end = System.nanoTime() - start;
		System.out.println("Time (ms): " + Double.valueOf(end) / 1_000_000);
		
		return quit;
	}

}
