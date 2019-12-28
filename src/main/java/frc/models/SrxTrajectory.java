package frc.models;


import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.modifiers.TankModifier;


//Combines left and right motion profiles in one object
public class SrxTrajectory {
	public boolean flipped;
	public boolean highGear;
	public SrxMotionProfile leftProfile;
	public SrxMotionProfile centerProfile;
	public SrxMotionProfile rightProfile;

	public SrxTrajectory() {
	}

	public SrxTrajectory(SrxMotionProfile left, SrxMotionProfile center, SrxMotionProfile right) {
		this.flipped = false;
		this.highGear = false;

		this.leftProfile = left;
		this.centerProfile = center;
		this.rightProfile = right;
	}
}
