package org.usfirst.frc.team4028.robot.auton;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Hashtable;

import org.usfirst.frc.team4028.robot.auton.PathBuilder.Waypoint;
import org.usfirst.frc.team4028.robot.auton.control.Path;
import org.usfirst.frc.team4028.robot.auton.motion.Translation;

import static org.usfirst.frc.team4028.robot.auton.PathBuilder.buildPathFromWaypoints;
import static org.usfirst.frc.team4028.robot.auton.PathBuilder.getStraightPathWaypoints;
import static org.usfirst.frc.team4028.robot.auton.PathBuilder.flipPath;
import static org.usfirst.frc.team4028.robot.auton.PathBuilder.reversePath;

public class Paths {
	private static Hashtable<Center, Path> centerPaths = new Hashtable<Center, Path>();
	private static Hashtable<Left, Path> leftPaths = new Hashtable<Left, Path>();
	private static Hashtable<Right, Path> rightPaths = new Hashtable<Right, Path>();
	
	public enum Center 
	{
		AUTO_RUN,
		AUTO_TURN,
		TEST
	}
public static Path autoRunPath;
public static Path autoTurnPath;
public static Path testPath;
	
	public enum Left {
		
	}

	
	public enum Right {
		
	}
	

	
	public static void buildPaths() {
		buildCenterPaths();
		buildLeftPaths();
		buildRightPaths();
	}
	
	public static Path getPath(Center pathName) {
		return centerPaths.get(pathName);
	}
	
	public static Path getPath(Left pathName) {
		return leftPaths.get(pathName);
	}
	
	public static Path getPath(Right pathName) {
		return rightPaths.get(pathName);
	}
	
	private static void buildCenterPaths() 
	{
		autoRunPath = buildPathFromWaypoints(getStraightPathWaypoints(new Translation(20, 46), 0, 60));
		centerPaths.put(Center.AUTO_RUN, autoRunPath);
		/*Path autoRunPath = buildPathFromWaypoints(0, Arrays.asList(
		new Waypoint(100,125,0,0),
		new Waypoint(140,60,40,20),
		new Waypoint(170,264,30,30),
		new Waypoint(225,264,0,40)
	));*/
		Path testPath = buildPathFromWaypoints(0, Arrays.asList(
			new Waypoint(100,100,0,0),
			new Waypoint(200,100,50,100),
			new Waypoint(260,200,0,100)));
		centerPaths.put(Center.TEST, testPath);
	}
	
	private static void buildLeftPaths() {
		

	}
	
	private static void buildRightPaths() {
		
	}
}