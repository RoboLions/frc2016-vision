package org.usfirst.frc.team1261.robot;

import java.io.IOException;
import java.time.LocalDateTime;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

/**
 * This is a short sample program demonstrating how to use the basic throttle
 * mode of the new CAN Talon.
 */
public class Robot extends IterativeRobot {

	// CANTalon motor;
	Servo motor;
	// use at resolution 
	//private final static String[] GRIP_ARGS = new String[] { "/usr/local/frc/JRE/bin/java", "-jar",
	//		"/home/lvuser/grip.jar", "/home/lvuser/project.grip" };

	public Robot() {
		motor = new Servo(9);
		// motor = new CANTalon(1); // Initialize the CanTalonSRX on device 1.
		// table = NetworkTable.getTable("GRIP/myContoursReport");
	}

	public void robotInit() {
	/*
		System.out.println("Running " + String.join(" ", GRIP_ARGS));
		try {
			Runtime.getRuntime().exec(GRIP_ARGS);
		} catch (IOException e) {
			System.out.println("Exception");
			e.printStackTrace();
		}
		*/
	}

	/**
	 * Runs the motor.
	 */
	public void teleopPeriodic() {
		double[] defaultValue = new double[] {};
		double[] centerYArray;
		double[] areaArray;
		double centerY = 120;
		double maxArea = 0;
		NetworkTable table;
		//while (isEnabled()) {
			table = NetworkTable.getTable("GRIP/myContoursReport");
			areaArray = table.getNumberArray("area", defaultValue);
			centerYArray = table.getNumberArray("centerY", defaultValue);
			
			for (int i = 0; i < areaArray.length; i++) {
				if (areaArray[i] > maxArea) {
					maxArea = areaArray[i];
					centerY = centerYArray[i];
				}
			}
			//System.out.print(LocalDateTime.now().toString());
			//System.out.print(height[0] + " ");
			//for (double cx : centerX) {
			//	System.out.print(cx + " ");
			//}
			//double angle =((height.length > 0 ? height[0] : 0)/215)*180;
			//System.out.print("angle: " + angle);
			System.out.print("centerY:" +  centerY);
			//motor.setAngle(angle);
			System.out.println();
			
	}
}
