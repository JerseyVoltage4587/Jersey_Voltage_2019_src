package frc.robot.paths;

import java.util.HashMap;

import frc.robot.Robot;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;

public class PathManager {
    PathWriter pw;
    public PathManager(){
        pw = new PathWriter();
    }

    public void writePaths(){
        Waypoint[] waypoints = {
			new Waypoint(0, 0, 0),
			new Waypoint(3, -1, 0),
		};
        pw.writePath("testPath", waypoints, false);
        
		pw.writePath("testPath1", waypoints, true);
    }

    private HashMap<String,Trajectory> trajMap = new HashMap<String,Trajectory>();
    public HashMap<String,Trajectory> getTrajMap(){
        return trajMap;
    }

    public void savePaths(){
        long startT = System.nanoTime();

        PathReader prTest = new PathReader("testPath");
        trajMap.put("testLeft", prTest.getLeftTraj());
        trajMap.put("testRight", prTest.getRightTraj());
        
        PathReader prTest1 = new PathReader("testPath1");
        trajMap.put("test1Left", prTest1.getLeftTraj());
        trajMap.put("test1Right", prTest1.getRightTraj());

        long endT = System.nanoTime();
        System.out.println("time saved by PathManager: "+(endT-startT)/1000000.0+" (ms)");
    }
}
