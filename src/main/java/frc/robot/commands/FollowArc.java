package frc.robot.commands;

import java.io.ObjectOutputStream;
import java.io.ObjectInputStream;
import java.io.FileOutputStream;
import java.io.FileInputStream;

import frc.models.BobTalonSRX;
import frc.models.SrxMotionProfile;
import frc.models.SrxTrajectory;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.Drivetrain;

import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Command;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;


public class FollowArc extends Command {

	/*
	* Key Parameters Required for Launching a Motion Profile.
	* Initialized by each Constructor.
	*/
	private int td; //Time Duration between each Trajectory Poiont, ie smaller is faster. use 10,20,30,..100 (ms)
	private boolean usePigeon = RobotMap.usePigeonDefault; /* else use encoder difference as auxPos feedback */
	private boolean inReverse = false;	
	private SrxTrajectory trajectoryToFollow = null;  //Constructors must provide this or a means to populate this (file(s) or waypoints)
	private String trajectoryFilename=null;
	private FollowArcTrajectory trajectoriesToJoin[] = null;
 	private Waypoint wayPoints[]=null;
	private String saveFilename=null;

	public static class FollowArcTrajectory {
		public String trajectoryFilename;
		public int td;
		public boolean usePigeonIMU;
		public boolean inReverse;
	}

	/*
	 * Motor Controllers
	 */
	private BobTalonSRX rightTalon = Robot.drivetrain.rightLead;
	private BobTalonSRX leftTalon = Robot.drivetrain.leftLead;
	
	/*
	 * Motion Profile Constants
	 */
	private int distancePidSlot = Drivetrain.HIGH_GEAR_PROFILE;
	private int rotationPidSlot = Drivetrain.ROTATION_PROFILE;
	private int kMinPointsInTalon = 5;

	/*
	 * Motion Profile managment variables
	 */
    private MotionProfileStatus status = new MotionProfileStatus();
    private SrxMotionProfile srxMP;
    private double startEncPosition,rightEnc,leftEnc,startDistance;
	private SetValueMotionProfile setValue = SetValueMotionProfile.Disable; //only Disable, Enable, or Hold.
	private Notifier mpBufferProcessor=null; // Notifier (runnable) Feed MP Executer runs at least twice as fast the fastest td (time duration) that you plan to use eg 10ms
	private int lastPointSent = 0;
	private double lastEncPos; //position of last point (even in an arraty of joined trajectories)

	/*
	 * General Flags used throughout
	 */
	private boolean isFinishedFlag = false;
	private boolean hasPathStarted;
	private int trajectoriesToJoinCount = 0;
	private int counter=0;
	private long timeMSBefore;
	private long timeMSAfter;



	private class MPBufferProcessor implements java.lang.Runnable {	
		TalonSRX talon;
		com.ctre.phoenix.motorcontrol.StickyFaults faults = new com.ctre.phoenix.motorcontrol.StickyFaults();
		
		public MPBufferProcessor(TalonSRX talon) {
			this.talon = talon;	

		}

		public void run() {
			talon.processMotionProfileBuffer();
						
			talon.getStickyFaults(faults);
			if (faults.UnderVoltage) {
				System.out.println("ERROR: Motion Profile Struggled On, UNDER VOLTAGE");
				//Robot.drivetrain.isClosedLoopFailure = true;
				//isFinishedFlag = true;
			}
			if (faults.RemoteLossOfSignal) {
				System.out.println("ERROR: Motion Profile Struggled On, REMOTE SIGNAL LOST");
				 //Robot.drivetrain.isClosedLoopFailure = true;
				//isFinishedFlag = true;
			}
			if (faults.SensorOutOfPhase) {
				System.out.println("ERROR: Motion Profile Struggled On, SENSOR OUT OF PHASE");
				 //Robot.drivetrain.isClosedLoopFailure = true;
				//isFinishedFlag = true;
			}
			if (faults.ResetDuringEn) {
				System.out.println("ERROR: Motion Profile Struggled On, RESET DURING EN");
				//Robot.drivetrain.isClosedLoopFailure = true;
				//isFinishedFlag = true;
			}
			System.out.printf(".");	
		}
	}
	
	
	public double loadMPAPIBuffer(int theLastPointSent,TalonSRX talon,SrxMotionProfile prof, int td, boolean flipped, double startPosition, boolean reversed,boolean isJoinerTrajectory) {
		int direction;
		double flipped180offset;
		double flippedByDegrees;
		double lastEncPosition;

		lastEncPosition = startPosition;
		//For reverse we need to invert the MP encoder position, velocity and flip the heading 180 degrees
		// nb: be careful to make sure starting heading is still in range 0-360 and apply the flippedByDerees to all points.
		direction = (reversed ? -1 : 1);
		if (Pathfinder.r2d(prof.points[0][3]) > 179) {
			flipped180offset = -180;	
		} else {
			flipped180offset = 180;	
		}
		flippedByDegrees = flipped ? flipped180offset : 0;
	
		if (theLastPointSent >= prof.numPoints) {
			return lastEncPosition;
		}
		while (!talon.isMotionProfileTopLevelBufferFull() && theLastPointSent < prof.numPoints) {
			TrajectoryPoint point = new TrajectoryPoint();

			/* for each point, fill our structure and pass it to API */
			/* To get point.postion from MP converted to meters, ask how many rotations in 1 meter = 2.08865 theoretically, but emperical measurement of 10 rotations to get rotations/meter is best */
			//point.position = this.startPosition + (this.direction * (this.prof.points[theLastPointSent][0] * RobotMap.SensorUnitsPerRotation * RobotMap.WheelRotationsPerMeter)); //Convert Revolutions to Units ;
			point.position = startPosition + (direction * (prof.points[theLastPointSent][0] * RobotMap.SensorUnitsPerMeter)); //Convert Revolutions to Units ;
			lastEncPosition=point.position;
			point.velocity = direction * (prof.points[theLastPointSent][1] * RobotMap.SensorUnitsPerRotation / 600.0); //Convert RPM to Units/100ms;
			point.timeDur = (int)td; //TrajectoryDuration.Trajectory_Duration_100ms;//.Trajectory_Duration_100ms;
			//point.auxiliaryPos = (flipped ? -1 : 1) * 10  * (prof.points[theLastPointSent][3] + startHeading);
			
			/* The auxiliaryPos uses 3600 for 360 degrees so multiply degrees by 10  */
			point.auxiliaryPos = 10  * (Pathfinder.r2d(prof.points[theLastPointSent][3])+flippedByDegrees);				
			point.profileSlotSelect0 = distancePidSlot;
			point.profileSlotSelect1 = rotationPidSlot;
			point.zeroPos = false;
			
			point.useAuxPID = true;
			
			point.isLastPoint = false;
			if (!isJoinerTrajectory && ((theLastPointSent + 1) == prof.numPoints)) {
				point.isLastPoint = true; /** set this to true on the last point */
			}
System.out.printf("FollowArc is filling MP trajectory for talon:%s startpos: %s point: position=%-8s velocity=%-4s auxPos=%-4s profileHeading(radians)=%-4s timedur=%-4s isLastPoint=%s\r",talon.getDeviceID(), startPosition, point.position, point.velocity,point.auxiliaryPos, prof.points[lastPointSent][3], point.timeDur,point.isLastPoint);
			talon.pushMotionProfileTrajectory(point);
			theLastPointSent++;
			hasPathStarted = true;
		}
		this.lastPointSent = theLastPointSent;
		return lastEncPosition;
	}		

	public FollowArc(SrxTrajectory trajectoryToFollow, boolean usePigeonIMU) {
		requires(Robot.drivetrain);
		this.td = 50;                                 //default time duration (ms) to take to complete each point (smaller=faster)
		this.usePigeon = usePigeonIMU;                //decide which aux method to use
		this.inReverse = false;                       //default is to play trajectory forwards 
		this.trajectoryToFollow = trajectoryToFollow; //we have a list of points to follow
		this.trajectoryFilename = null; //not reading trajectory points from a file
		this.trajectoriesToJoin = null; //not joining any trajectories
        this.wayPoints = null;          //not generating trajectory from waypoints
		this.saveFilename = null;	    //not saving any new generated trajectories
	}

	public FollowArc(SrxTrajectory trajectoryToFollow) {
        this(trajectoryToFollow,RobotMap.usePigeonDefault); //use pigeon default
	}

	
	public FollowArc(String trajectoryFilename) {
		this(trajectoryFilename,RobotMap.usePigeonDefault); //use pigeon default
	}
	
	public FollowArc(String trajectoryFilename, boolean usePigeonIMU) {
		this(trajectoryFilename,50,usePigeonIMU,false);
	}
	
	public FollowArc(String trajectoryFilename, int td, boolean usePigeonIMU, boolean reverse) {
		requires(Robot.drivetrain);
		System.out.println("FollowArc Constructor");
		this.td = td;
		this.usePigeon = usePigeonIMU;
		this.inReverse = reverse;
		this.trajectoryToFollow = null; //not providing a direct list of trajectory points
		this.trajectoryFilename = trajectoryFilename;
		this.trajectoriesToJoin = null; //not joining any trajectories together
		this.wayPoints = null;          //not generating trajectory from waypoints
		this.saveFilename = null;	    //not generating and saving and new trajectories

/*		if (readTrajectory(trajectoryFilename)) {
        	this.trajectoryToFollow.highGear = false;
        	this.trajectoryToFollow.flipped = false;    
        } else {
			System.out.println("We had no Trajectory");
			this.isFinishedFlag = true;
		}*/
	}
	

	
	public FollowArc(FollowArcTrajectory faTrajectory) {
		this(faTrajectory.trajectoryFilename, faTrajectory.td, faTrajectory.usePigeonIMU, faTrajectory.inReverse);	
	}

	public FollowArc(FollowArcTrajectory trajectoriesToJoin[]) {
		requires(Robot.drivetrain);
		System.out.println("FollowArc Constructor");
		this.td = trajectoriesToJoin[0].td;
        this.usePigeon = trajectoriesToJoin[0].usePigeonIMU;
        this.inReverse = trajectoriesToJoin[0].inReverse;
        this.trajectoryToFollow = null; //not providing a direct list of trajectory points
		this.trajectoryFilename = trajectoriesToJoin[0].trajectoryFilename; //not reading trajectory points from a file
		this.trajectoriesToJoin = trajectoriesToJoin;  //the array of trajectories to join
		this.wayPoints = null;          //not generating trajectory from waypoints
        this.saveFilename = null;       //not generating and saving and new trajectories
 	}
	
	public FollowArc(Waypoint[] wayPoints) {
	    this(wayPoints,50,RobotMap.usePigeonDefault,false,null); //use pigeon default
	}    

	public FollowArc(Waypoint[] wayPoints, int td, boolean usePigeonIMU, boolean reverse, String saveFilename) {
		System.out.println("Follow Arc Contructor");
		this.td = 50;
        this.usePigeon = usePigeonIMU;
        this.inReverse = reverse;
        this.trajectoryToFollow = null; // we will generate the trajectory once the Command has been started
		this.trajectoryFilename = null; //not reading trajectory points from a file
		this.trajectoriesToJoin = null; //not joining any trajectories
		this.wayPoints = wayPoints;     // we will use these waypoints to generate the trajectory
        this.saveFilename = saveFilename; //we will save the trajectory once we have created it (so it's faster next time)
    }

    
	public boolean saveTrajectory(String toFilename) {
		try {
			FileOutputStream fos = new FileOutputStream(toFilename);
			ObjectOutputStream oos = new ObjectOutputStream(fos);
			oos.writeObject(this.trajectoryToFollow.centerProfile.points);
			oos.close();
		} catch (Exception e) {
			System.out.printf("Error: Failed to Save to: %s\r",toFilename);
			e.printStackTrace();
			return false;
		}
		return true;
	}
    
	public boolean readTrajectory(String fromFilename) {
		try {
			this.trajectoryFilename=fromFilename;
			FileInputStream fis = new FileInputStream(fromFilename);
			ObjectInputStream ois = new ObjectInputStream(fis);
			this.trajectoryToFollow = new SrxTrajectory();
			this.trajectoryToFollow.centerProfile = new SrxMotionProfile();
			this.trajectoryToFollow.centerProfile.points = (double[][])ois.readObject();
			int i=0;
			while (((this.trajectoryToFollow.centerProfile.points[i][0] != 0.0) ||
				    (this.trajectoryToFollow.centerProfile.points[i][1] != 0.0) ||
					(this.trajectoryToFollow.centerProfile.points[i][3] != 0.0))
					&& i < SrxMotionProfile.MAX_POINTS) {
				i++;			
			}	
			this.trajectoryToFollow.centerProfile.numPoints = i; 
			this.trajectoryToFollow.flipped = false;
			this.trajectoryToFollow.highGear = false;
		} catch (Exception e) {
			System.out.printf("Error: Failed to read from: %s\r", fromFilename);
			//e.printStackTrace();
			this.trajectoryToFollow = null;
			return false;
		}
		return true;
	}
	
	public boolean hasTrajectory() {
		return (this.trajectoryToFollow != null);
	}
	
	// Called just before this Command runs the first time
	protected void initialize() {
		System.out.println("FollowArc Starting");
		/*
		 * Mandatory Constructor Variables (have been set up for use)
		 * ==========================================================
		 * td                    - time duration (ms) to take to complete each point (smaller=faster)
		 * usePigeon             - which aux method to use pigeon or encoder difference
		 * inReverse             - how to play the trajectoryToFollow (forwards or in reverse)
		 *                         nb: reverse is achieved by inverting encoder and velocities and flipping the headings 180 degrees
		 * trajectoryToFollow    - a list of trajectory points to follow
		 * trajectoryFilename    - trajectory points filename to use
		 * trajectoriesToJoin    - join multiple trajectory point files together (also honours each trajectories td, usePigeon,inReverse)
		 * wayPoints             - generate a trajectory from waypoints
		 * saveFilename          - save any new generated trajectories using this filename
		 */


        /* Initialize General use variables */
		this.isFinishedFlag=false;
		this.status.activePointValid=false;
		this.status.isLast = false;
		this.mpBufferProcessor = null;
		this.trajectoriesToJoinCount = 0;
		this.lastPointSent = 0;
		resetTalon(rightTalon, ControlMode.PercentOutput, 0);
	  	resetTalon(leftTalon, ControlMode.PercentOutput, 0);


		/*
		 * Generate Trajectory - note that it takes a fair amount of time to this on the Roborio.
		 *                       (approx 0.9 - 1.3 seconds)
		 */
		if (this.wayPoints != null) {
	    	Trajectory.Config trajectoryConfig;
	    	Trajectory trajectory; 
			
	        /* Calculate the Trajectory */
	    	trajectoryConfig = new Trajectory.Config(Trajectory.FitMethod./*HERMITE_QUINTIC*/HERMITE_CUBIC, 
			 		Trajectory.Config.SAMPLES_HIGH, 0.05, 3.5, 2.0, 60.0);  
		
	    	this.timeMSBefore = System.currentTimeMillis();
	    	trajectory = Pathfinder.generate(wayPoints, trajectoryConfig);
	 System.out.printf("trajectory points calculated: %s\r", trajectory.length());	    
	    	this.timeMSAfter = System.currentTimeMillis();
	    	System.out.printf("TIMER: Pathfinder.generate - X: %s Y:%s generate (ms): %s\r",wayPoints[1].x,wayPoints[1].y,(timeMSAfter-timeMSBefore));   	
	    	this.srxMP = new SrxMotionProfile(trajectory);
			this.trajectoryToFollow = new SrxTrajectory();
			this.trajectoryToFollow.centerProfile = this.srxMP;
			this.trajectoryToFollow.highGear = false;
			this.trajectoryToFollow.flipped = false;
	System.out.printf("trajectory points in trajectoryToFollow:: %s\r", this.trajectoryToFollow.centerProfile.numPoints);
			if ((this.saveFilename == null) || !saveTrajectory(saveFilename)){
			    this.isFinishedFlag = true;
				System.out.printf("The generated trajectory could not be saved: %s\r",(this.saveFilename == null) ? "No Filename" : this.saveFilename);
			} 
	        this.trajectoryToFollow = null;
	        this.trajectoryFilename = saveFilename;  //so that the trajectory file we just created will be used below
		}
		
		/*
		 * A this point We must have a trajectoryToFollow or a trajectory filename to read or we quit.
		 */

		if ((this.trajectoryToFollow != null) || (this.trajectoryFilename != null) || (this.trajectoriesToJoin != null)) {
			if (this.trajectoryFilename != null) {
               readTrajectory(this.trajectoryFilename);
			}

			//trajectoryFilename is loaded by trajectoriesToJoin version of constructer so above will have done the job
			if (this.trajectoriesToJoin != null) {
				readTrajectory(this.trajectoriesToJoin[0].trajectoryFilename);
			}
 			
			if ((this.trajectoryToFollow == null) || 
			    (this.trajectoryToFollow.centerProfile.points == null) ||
				(this.trajectoryToFollow.centerProfile.numPoints < this.kMinPointsInTalon)) {
					this.isFinishedFlag=true;
					System.out.println("ERROR: Attempted to follow a Trajectory, but failed");
			}
		} else {
			isFinishedFlag = true;
			System.out.println("ERROR: No Trajectory to follow");
		}
		
		if (!isFinishedFlag) {

			/* 
			 * Initialize the drive train to a known state (setupDriveTrain) 
			 *  -  we're being careful to make sure that robot doesn't not respond to our encoder changes until we have 
			 *     set up the sensors to represent an initial angle which matches our first trajectory heading 
			 */
			//this.timeMSBefore = System.currentTimeMillis();		
			//Robot.drivetrain.setupDrivetrain();
			//this.timeMSAfter = System.currentTimeMillis();
            //System.out.printf("TIMER: setupDrivetrain: %s ms\r", this.timeMSAfter-this.timeMSBefore);	    
			

			/*
			 * Prepare the drivetrain into Pigeon or Encoder Difference mode.
			 *   NB: Difference and Sum for calculating distance and aux position doesn't come into effect until we are in 
			 *       Follower Mode (maybe we need to set the master Talon in control mode Motion Profile too?).  
			 */
			if (this.usePigeon)
				Robot.drivetrain.setNavModePigeonAngle();
			else	
				Robot.drivetrain.setNavModeEncoderDiffAngle();

			System.out.println("FollowArc Started");	
			//System.out.printf("Last Angle:%s\r", Robot.drivetrain.getNavAngle());

			/*
			 * Set up our sensors so that the calculated angle will match the expected heading of the first point of the trajectory 
		     *  (otherwise it will attempt to jump around to correct this, usually not desirable)
			 */
            setUpSensorPositionForTrajectoryInitialHeading();
			
			//firstTime=true;  /* indicates that we MP has not yet commenced, need to wait until getDistance == startDistance */
			
			/* used for displaying output once every N execute cycles, reducing traffic on the CAN bus talking to Talons */
			counter=0; 
			
			
			/*if (trajectoryToFollow.highGear) {
				Robot.pneumatics.drivetrainShiftUp();
			}else {
				//Robot.pneumatics.drive	trainShiftDown();
				Robot.pneumatics.drivetrainShiftUp();
			}*/
/*
			if (Robot.m_oi.driveStick.getThrottle() > 0) {  //SAFETY ON +1 towards minus side   (-1 towards plus sign)
				System.out.printf("Throttle (Safety) :%s\r", Robot.m_oi.driveStick.getThrottle());
				System.out.printf("SAFETY MODE ON (drive is off)\r");
				int i;
				for (i=0; i<this.trajectoryToFollow.centerProfile.numPoints; i++) {
 				System.out.printf("point[%s]: pos:%s: vel:%s, heading:%s\r",i,
									this.trajectoryToFollow.centerProfile.points[i][0],
									this.trajectoryToFollow.centerProfile.points[i][1],
									Pathfinder.r2d(this.trajectoryToFollow.centerProfile.points[i][3]));	
				}
				this.isFinishedFlag = true;
			} else {
				System.out.printf("Safety MODE OFF (drive on):%s\r",Robot.m_oi.driveStick.getThrottle());
			*/
				//setUpTalon(this.leftTalon);
				setUpTalon(this.rightTalon);
		
				setValue = SetValueMotionProfile.Disable;

				rightTalon.set(ControlMode.MotionProfileArc, setValue.value);	
				leftTalon.follow(rightTalon, FollowerType.AuxOutput1);
				
				if (this.trajectoryToFollow != null) {
					double encPos=0;
					lastEncPos = 0;
					if (trajectoriesToJoin != null) {
						System.out.printf("trajectoriesToJoin.length: %s\r",trajectoriesToJoin.length);
					    for (int x=0;x<trajectoriesToJoin.length;x++) {
							if (x>0) {
							  	readTrajectory(this.trajectoriesToJoin[x].trajectoryFilename);
							}
							this.lastPointSent = 0; /* future: if we have problems exceeding 2048 of API Buffer we may need to loop feed this */

						    encPos = loadMPAPIBuffer(this.lastPointSent,this.rightTalon, this.trajectoryToFollow.centerProfile, this.trajectoriesToJoin[x].td,
							        this.trajectoriesToJoin[x].inReverse,
									(x==0)?this.startEncPosition:lastEncPos,
									this.trajectoriesToJoin[x].inReverse,
									(x<(trajectoriesToJoin.length-1))?true:false);
							lastEncPos = encPos;	
							System.out.printf("lastPointSent: %s numPoints: %s\r",lastPointSent, this.trajectoryToFollow.centerProfile.numPoints);
						}			
					} else {
						this.lastPointSent = 0; /* future: if we have problems exceeding 2048 of API Buffer we may need to loop feed this */
						lastEncPos = loadMPAPIBuffer(this.lastPointSent,this.rightTalon, this.trajectoryToFollow.centerProfile, this.td,  
							this.inReverse,
							this.startEncPosition,
							this.inReverse,false);
					}	

					this.mpBufferProcessor = new Notifier(
							new MPBufferProcessor(this.rightTalon));
					this.mpBufferProcessor.startPeriodic(0.015);		

					

				}
			//}     
		}
	}

	/* 
	 * setUpSensorPositionForTrajectoryInitialHeading()
	 * 
	 * Before launching a Motion Profile it's good to set your sensors to match the beginning point of your Motion Profile.
	 * 
	 * Runs the following method:
	 * setNavAngle() - Set the pigeon or encoders to the heading of the first point of the current trajectoryToFollow
	 * 
	 * Prepares the following variable:
	 * startEncPosition - Also sets the startEncPosition to the current value that will be calculated when in Motion Pofile control mode.
	 * 
	 */
	private void setUpSensorPositionForTrajectoryInitialHeading() {
		if (!this.usePigeon) {
			Robot.drivetrain.setNavAngle(Pathfinder.r2d(this.trajectoryToFollow.centerProfile.points[0][3]));				
		/*
			System.out.printf("MP Start Pos:   %s\r" ,this.trajectoryToFollow.centerProfile.points[0][0]);
			System.out.printf("MP Start Angle: %s (%s)\r",this.trajectoryToFollow.centerProfile.points[0][3],Pathfinder.r2d(this.trajectoryToFollow.centerProfile.points[0][3]));
			System.out.printf("Left (selSensor) Encoder:   %s\r", Robot.drivetrain.leftLead.getSelectedSensorPosition());
			System.out.printf("Right (raw) Encoder:  %s\r", Robot.drivetrain.rightLead.getSensorCollection().getQuadraturePosition());
			System.out.printf("right getPrimarySensorPosition(): %s\r", Robot.drivetrain.getDistance());
			System.out.printf("Reverse MP:     %s\r",inReverse);
			System.out.printf("Adding this to MP Pos (getDistance-[reverse MP Start Pos]): %s\r",Robot.drivetrain.getDistance()- (inReverse?(this.trajectoryToFollow.centerProfile.points[0][0]):0));
		*/	
			
			rightEnc = (int)Math.round((Pathfinder.r2d(this.trajectoryToFollow.centerProfile.points[0][3])/360)*RobotMap.EncoderPulsesToRotateBot360DegreesOnTheSpot);
			leftEnc=0;
			
			/* 
			* inReverse the MP is reversed, and starts from an encoder position of the last point on the original MP,
			*           in this case, we need to give a startEncPosition with that subtracted 
			*/
			startDistance = Math.round(((leftEnc + rightEnc)*0.5));
			startEncPosition=startDistance - ((!usePigeon) && inReverse?(this.trajectoryToFollow.centerProfile.points[0][0]):0);
			System.out.printf("CALCULATED: right getPrimarySensorPosition(): %s\r", (leftEnc + rightEnc)*0.5);
			System.out.printf("CALCULATED: Adding this to MP Pos (getDistance-[reverse MP Start Pos]): %s\r",startEncPosition);
			
		} else {
			/* 
			* The Sum and Difference used systems don't work unless you are Follower and master is in Motion Profile control Mode
			* so we have to do the calulations manually so that the startEncPosition will represent
			* the distance calulated once the MP control mode is started.
			*/
			
			startEncPosition = 0.5 *(Robot.drivetrain.rightLead.getSensorCollection().getQuadraturePosition() +
							Robot.drivetrain.leftLead.getSelectedSensorPosition(0));
			System.out.printf("Setting start position to current Sensor Sum - calc from raw %s\r",startEncPosition);
			/* need to set our angle to equiv angle, bound between 0-360 before launching an MP */
			double angle = Robot.drivetrain.getNavAngle();
			Robot.drivetrain.setNavAngle(Robot.drivetrain.bound360(angle));
			System.out.printf("Setting Nav Angle from %s to %s (in range 0-360)\r",angle, Robot.drivetrain.bound360(angle));
		}
    }



	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		if (!isFinishedFlag) {
			rightTalon.getMotionProfileStatus(status);
			if (status.isUnderrun) {
				// if either MP has underrun, stop both
				System.out.println("Motion profile has underrun!");
				setValue = SetValueMotionProfile.Disable;
			} else if (status.btmBufferCnt > kMinPointsInTalon)  {
				setValue = SetValueMotionProfile.Enable;
			} else if (status.activePointValid && status.isLast) {
				// if both profiles are at their last points, hold the last point
				setValue = SetValueMotionProfile.Hold;
				this.isFinishedFlag = true;
				//System.out.printf("status.activePointValid = true and status.isLast = true   status.btmBufferCnt: %s\r",status.btmBufferCnt);					
			} else { 
				System.out.printf("MP Low-Level Buffer Loading Up => status.isUnderrun:%s, status.btmBufferCnt: %s, status.activePointValid: %s, status.isLast:%s \r",status.isUnderrun, status.btmBufferCnt, status.activePointValid, status.isLast);
			}
				//counter++;
				//if (counter % 25 == 0) { //updates dashboard byreading from encoders every half a second
					/*
					SmartDashboard.putNumber("**Distance ClosedLoop Error", rightTalon.getClosedLoopError());
					SmartDashboard.putNumber("**Distance ClosedLoop Target", rightTalon.getClosedLoopTarget(distancePidSlot));				
					SmartDashboard.putString("**Distance LR Encoder SUM (avg) Position", rightTalon.getPrimarySensorPosition() + " " + 0.5*(rightTalon.getSensorCollection().getQuadraturePosition()+leftTalon.getSensorCollection().getQuadraturePosition()));
					SmartDashboard.putNumber("**Rotation ClosedLoop Error", rightTalon.getSecondaryClosedLoopError());
					SmartDashboard.putNumber("**Rotation ClosedLoop Target", rightTalon.getClosedLoopTarget(rotationPidSlot));			
					SmartDashboard.putNumber("**Rotation LR Encoder DIFF Position", Robot.drivetrain.getNavAngle());
					SmartDashboard.putNumber("**MP Active Trajectory Heading: %s", rightTalon.getActiveTrajectoryHeading());
					SmartDashboard.putNumber("**MP Active Trajectory Position: %s", rightTalon.getActiveTrajectoryPosition());
					SmartDashboard.putNumber("**MP Active Tracjectory Velocity: %s", rightTalon.getActiveTrajectoryVelocity());
					*/
					
					//SmartDashboard.putNumber("Calculated Angle", angle);
					//angle = rightLead.getSecondarySensorPosition()/10.0;
					//SmartDashboard.putNumber("Secondary Sensor Angle", angle);
			
				//}
		}	
		/*
		 * Set master Talon to MotionProfileArc (Enable, Disable or Hold)
		 */ 	
		rightTalon.set(ControlMode.MotionProfileArc, setValue.value);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		if (this.isFinishedFlag) {
			System.out.println("Finished trajectory");
		} else {
			//System.out.println("INCOMPLETE TRAJECTORY");
		}
		return this.isFinishedFlag;
	}

	// Called once after isFinished returns true
	protected void end() {
		//this.isFinishedFlag=true;
        //this.trajectoryToFollow = null;

		if (this.mpBufferProcessor != null) {
			System.out.println("end: Stopping 5ms Notifier (pushing MP points to MP Executer)");
			this.mpBufferProcessor.stop();
			this.mpBufferProcessor.close();
		}	
		resetTalon(rightTalon, ControlMode.PercentOutput, 0);
	  	resetTalon(leftTalon, ControlMode.PercentOutput, 0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		//this.isFinishedFlag = true;
		//this.trajectoryToFollow = null;
		if (this.mpBufferProcessor != null) {
			System.out.println("end: Stopping 5ms Notifier (pushing MP points to MP Executer)");
			this.mpBufferProcessor.stop();   
			this.mpBufferProcessor.close();  
		}
		resetTalon(rightTalon, ControlMode.PercentOutput, 0);
		resetTalon(leftTalon, ControlMode.PercentOutput, 0);
		
    }

	// set up the talon for motion profile control
	private void setUpTalon(TalonSRX talon) {
		talon.clearMotionProfileTrajectories();
		talon.changeMotionControlFramePeriod(15);
		talon.clearMotionProfileHasUnderrun(0);
		talon.setStatusFramePeriod(StatusFrame.Status_12_Feedback1,20);
		talon.setStatusFramePeriod(StatusFrame.Status_10_Targets,20);
		talon.setStatusFramePeriod(StatusFrame.Status_17_Targets1,20);
	}
	
	// set the to the desired controlMode
	// used at the end of the motion profile
	private void resetTalon(TalonSRX talon, ControlMode controlMode, double setCMValue) {

		talon.clearMotionProfileTrajectories();
		talon.clearMotionProfileHasUnderrun(0);
		talon.changeMotionControlFramePeriod(10);
		talon.set(controlMode, setCMValue);
	}
}