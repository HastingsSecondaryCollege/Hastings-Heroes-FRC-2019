package frc.models;

import frc.robot.Constants;

//import com.ctre.phoenix.motion.TrajectoryPoint.TrajectoryDuration;

import edu.wpi.first.wpilibj.DriverStation;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.modifiers.TankModifier;


//Generic Motion Profile Class
public class SrxMotionProfile {

	public static final int POINT_POSITION = 0;
	public static final int POINT_VELOCITY = 1;
	public static final int POINT_TIMEDUR = 2;
	public static final int POINT_HEADING = 3;
	public static final int POINT_ACCELERATION = 4;
	public static final int POINT_JERK = 5;
	
	public static final int MAX_POINT_PARAMS = 6;
	public static final int MAX_POINTS = 1000;
	
 
	
	public int numPoints;
	public boolean isReversed=false;
	
	// Position (rotations) Velocity (RPM) Duration (ms)
	public double[][] points = new double[MAX_POINTS][MAX_POINT_PARAMS];

	public SrxMotionProfile() {
	  /* Initialize the multidimensional array of doubles */
	  for (int x = 0; x< MAX_POINTS; x++)
          for (int y = 0; y <MAX_POINT_PARAMS; y++)
	         points[x][y] = 0.0;	
	}

	public SrxMotionProfile(int numPoints, double[][] points) {
		this.numPoints = numPoints;
		this.points = points;
	}
	
	public SrxMotionProfile(Trajectory trajectory) {
  
		int x;
		this.numPoints = trajectory.length();
		for (x=0; ((x < this.numPoints) && (x < MAX_POINTS)); x++) {
		    this.points[x][POINT_POSITION] = trajectory.segments[x].position; //* Constants.kSensorUnitsPerRotation; //Convert Revolutions to Units
		    this.points[x][POINT_VELOCITY] = trajectory.segments[x].velocity; //* Constants.kSensorUnitsPerRotation / 600.0; //Convert RPM to Units/100ms
		    this.points[x][POINT_TIMEDUR] = trajectory.segments[x].dt;
		    this.points[x][POINT_HEADING] = trajectory.segments[x].heading;
System.out.printf("pos: %-5s vel:%-5s td:%s heading:%-5sf(%-5s)\r",this.points[x][POINT_POSITION],this.points[x][POINT_VELOCITY],this.points[x][POINT_TIMEDUR],this.points[x][POINT_HEADING],Pathfinder.r2d(this.points[x][POINT_HEADING]));		
		}
	}	
	
	public void reversed() {
		double[] pointbuff = new double[MAX_POINT_PARAMS];
		
		if (this.isReversed)
            this.isReversed = false;
		else
			this.isReversed = true;
		
	    for (int x=0; x<(int)(this.numPoints); x++) {
	  	       System.out.printf("FORWARD pos: %s vel:%s td:%s heading:%s\r",this.points[x][POINT_POSITION],this.points[x][POINT_VELOCITY],this.points[x][POINT_TIMEDUR],Pathfinder.r2d(this.points[x][POINT_HEADING]));
	    }
	    for (int x=0,y=this.numPoints-1; x<(int)(this.numPoints/2); x++,y--) {
	    		pointbuff[POINT_POSITION] = this.points[x][POINT_POSITION]; 
	    		pointbuff[POINT_VELOCITY] = this.points[x][POINT_VELOCITY];
	    		pointbuff[POINT_TIMEDUR] = this.points[x][POINT_TIMEDUR];
	    		pointbuff[POINT_HEADING] = this.points[x][POINT_HEADING];
	    		this.points[x][POINT_POSITION] = this.points[y][POINT_POSITION];
			    this.points[x][POINT_VELOCITY] = this.points[y][POINT_VELOCITY];
			    this.points[x][POINT_TIMEDUR] = this.points[y][POINT_TIMEDUR];
			    this.points[x][POINT_HEADING] = this.points[y][POINT_HEADING];
			    
			    this.points[y][POINT_POSITION] = pointbuff[POINT_POSITION]; 
	    		this.points[y][POINT_VELOCITY] = pointbuff[POINT_VELOCITY];
	    		this.points[y][POINT_TIMEDUR] = pointbuff[POINT_TIMEDUR]; 
	    		this.points[y][POINT_HEADING] = pointbuff[POINT_HEADING]; 

	    }
	    this.points[this.numPoints-1][POINT_VELOCITY] = 0;
	    for (int x=0; x<(int)(this.numPoints); x++) {
  	       System.out.printf("REVERSED pos: %s vel:%s td:%s heading:%s\r",this.points[x][POINT_POSITION],this.points[x][POINT_VELOCITY],this.points[x][POINT_TIMEDUR],Pathfinder.r2d(this.points[x][POINT_HEADING]));
    	}
	}
}
