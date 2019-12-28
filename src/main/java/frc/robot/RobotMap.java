/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import edu.wpi.first.wpilibj.Joystick;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;

/****** DELETE IF POSSIBLE 
 * import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.modifiers.TankModifier;

//import frc.robot.commands.SmartDash;
*/

/**
 * The RobotMap is a mapping tool from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around. 
 */
public class RobotMap {

  /***************************************** PORTS *****************************************/

  public static final int TalonBackLeft = 1;           // CAN ID (CANNOT BE 0)
  public static final int TalonFrontLeft = 2;          // CAN ID
  public static final int TalonFrontRight = 3;         // CAN ID
  public static final int TalonBackRight = 4;          // CAN ID
  public static final int TalonElevator = 5;           // CAN ID
  public static final int TalonExtensionRail = 6;     // CAN ID
  public static final int TalonHabLift = 7;          // CAN ID 
  public static final int VictorIntake = 8;            // CAN ID
  public static final int VictorHabDrive = 9;          // CAN ID 
  public static final int VictorSolenoidHatch = 10;    // CAN ID  (SOLENOID FOR HATCH)
  public static final int VictorDisableElevatorTop = 11;

  public static final int SparkLED = 0;               // PWM PORT  (LED)

  public static Joystick driveStick = new Joystick(0);      // PC PORT
  public static Joystick xboxController = new Joystick(1);  // PC PORT
  public static Joystick testStick = new Joystick(2);

  public static OI m_oi; // Operator Interface (NOT A PORT) 

  /***************************************** CONSTANTS *****************************************/
 
  // PID
  public static final double ELEVATOR_PID_P = 2.0;
  public static final double ELEVATOR_PID_I = 0.0;
  public static final double ELEVATOR_PID_D = 0.0;
  public static final double EXTENSION_RAIL_PID_P = 0.5;
  public static final double EXTENSION_RAIL_PID_I = 0.0;
  public static final double EXTENSION_RAIL_PID_D = 0.0;
  public static final double HABLIFT_PID_P = 0.5;
  public static final double HABLIFT_PID_I = 0.0;
  public static final double HABLIFT_PID_D = 0.0;
  
  // Climb Heighs
  public static final int CLIMB_LEVEL_TWO = 2; // (position index)
  public static final int CLIMB_LEVEL_THREE = 3;  
  
  // Hab Drive
  public static final double HAB_DRIVE_TIMEOUT = 5.0; // (double seconds)
  public static final double HAB_DRIVE_PRE_SPEED = 0.3; // (double speed)
  public static final double HAB_DRIVE_POST_SPEED = 0.4; // (double speed)
  public static final double HAB_DRIVETRAIN_POST_SPEED = -0.2; // was positive
  public static final double HAB_DRIVETRAIN_PRE_SPEED = -0.20; // was positive
  public static final double HAB_DRIVETRAIN_ANGLE = 0;
  // public static final double HAB_ELEVATOR_SPEED = 0.0; // (double speed)
  // public static final double HAB_LIFT_SPEED = 0.1; // (double speed)   
  // public static final double HAB_LIFT_RETRACT_SPEED = -0.1;

  // Hab Lift
  public static final double HAB_CLIMB_TIME = 0.5; 
  public static final double HAB_CLIMB_RETRACT_TIME = 0.5;     
  public static final double HAB_CLIMB_DELAY = 2.0;       // to be tested
  public static final double ENCODER_HAB_LIFT_RETRACTED = 0.0; // (encoder value)
  public static final double ENCODER_HAB_LIFT_LEVEL_TWO = -4737; //-4955; // comp bot -4955; // 2nd bot -5090.0; // comp bot -15406. Level 3 -15840
  public static final double ENCODER_HAB_LIFT_LEVEL_THREE = -15457; //-15620; // compt bot -15620; // 2nd bot -15840.0; // comp bot -15406. Level 3 -15840
  public static final double HABLIFT_MAX_FORWARD_SPEED = 0.75;  // was 0.7 but back was to slow   
  public static final double HABLIFT_MAX_REVERSE_SPEED = -0.75; //was 0.7 but back was to slow  
  public static final double HAB_DRIVE_SLOW_TIME = 2.0;
  public static final double HAB_INIT_POWER = 0.15; // was 0.1
  public static final double HAB_LIFT_CLIMB_TOLERANCE = 500;
  public static final double HAB_LIFT_FINAL_DRIVE_TOLERANCE = 200;

  // Extension Rail
  public static final double RAIL_RETRACTED = 0;
  public static final double RAIL_EXTENDED = -280893; // comp bot -280893; //2nd bot -277143
  public static final double RAIL_TOLERANCE = 100;  // Is extension rail on target
  public static final double RAIL_TOLERANCE_ELEVATOR_MOVE = 221823; // comp bot 221823; // 2nd bot 87987; // If the extension rail is farther than this constant from the extended position, the elevator will not move
  public static final double EXTENSION_RAIL_INIT_POWER = -  -0.3;
  public static final double EXTENSION_RAIL_MAX_FORWARD_SPEED = 1.0;     
  public static final double EXTENSION_RAIL_MAX_REVERSE_SPEED = -1.0;  
  
  // Elevator
  public static final double DISABLE_ELEVATOR_TOP_TRIGGER_DELAY = 2.0;
  public static final double DISABLE_ELEVATOR_TOP_TRIGGER_DISTANCE = 0.5;  
  public static final double DISABLE_ELEVATOR_TOP_POWER = 0.3; 
  public static final double DISABLE_ELEVATOR_TOP_DURATION = 0.5; 
  public static final double ELEVATOR_TOLERANCE = 525; // comp bot 525.0; // 2nd bot 1761.0;  // Can extension rail move was 1000
  public static final double ELEVATOR_MAX_FORWARD_SPEED = 0.8;   // GO DOWN
  public static final double ELEVATOR_MAX_REVERSE_SPEED = -1.0;  // GOING UPPPP  
  public static final double ELEVATOR_MAX_CLIMB_SPEED = 0.8; // WAS 0.7
  public static final double ELEVATOR_CLIMB_TOLERANCE = 100;
  public static final int LEVEL_ZERO_GROUND = 1; // (position index)
  public static final int LEVEL_ONE_HATCH = 2;  
  public static final int LEVEL_ONE_CARGO_CARGOSHIP = 3;
  public static final int LEVEL_ONE_CARGO_ROCKET = 4;
  public static final int LEVEL_TWO_HATCH = 5;
  public static final int LEVEL_TWO_CARGO = 6;
  public static final int LEVEL_THREE_HATCH = 7;
  public static final int LEVEL_THREE_CARGO = 8;
  public static final int LEVEL_TWO_CLIMB = 9;
  public static final int LEVEL_THREE_CLIMB = 10;
  public static final int LEVEL_DRIVE_WITH_CARGO = 11;
  public static final double DIFFERENCE_BETWEEN_CLIMB_AND_LOW_HATCH = 0; // was 130
  public static final double ENCODER_LEVEL_TEST = 0; // (Test)
  public static final double ENCODER_LEVEL_AFTER_CLIMB = 0;
  public static final double ENCODER_LEVEL_ZERO_GROUND = (0 - DIFFERENCE_BETWEEN_CLIMB_AND_LOW_HATCH); // WAS -194; // (encoder value)
  public static final double ENCODER_LEVEL_ONE_HATCH = (0 - DIFFERENCE_BETWEEN_CLIMB_AND_LOW_HATCH); // WAS-194; // descended to encoder count 39
  // bottom of top hatch panel patch to bottom of ball holder frame 87mm
  public static final double ENCODER_LEVEL_ONE_CARGO_CARGOSHIP = -8992; // comp bot -8798; // 2nd bot -8588; // was 8743 (COMP BOT)  -9083(PRAC BOT)
  // Top of Cargoshop to bottom of ball holder frame 73mm
  public static final double ENCODER_LEVEL_ONE_CARGO_ROCKET = -5600;// -5611; // comp bot-4777; // 2nd bot -4875; // was 5463 (COMP BOT) -5439 2nd Bot
  // bottom of low rocket cargo hole to bottom of ball holder frame 80mm 
  public static final double ENCODER_LEVEL_TWO_HATCH = (-6900 - DIFFERENCE_BETWEEN_CLIMB_AND_LOW_HATCH);//(-7161 - DIFFERENCE_BETWEEN_CLIMB_AND_LOW_HATCH); // comp bot -6883; // 2nd bot -6875; // was 7525 (COMP BOT) 6875 2nd bot -7315
  // bottom of top hatch panel patch to bottom of ball holder frame 67mm
  public static final double ENCODER_LEVEL_TWO_CARGO = -12412; // comp bot -12237; // 2nd bot -12319;  // swas 12082 (COMP BOT) 2nd bot -12921
  // bottom of middle rocket cargo hole to bottom of ball holder frame 84mm 
  public static final double ENCODER_LEVEL_THREE_HATCH =  (-13749 - DIFFERENCE_BETWEEN_CLIMB_AND_LOW_HATCH); //(-13899 - DIFFERENCE_BETWEEN_CLIMB_AND_LOW_HATCH); // comp bot -13621; // 2nd bot -13562; // was 14380 (COMP BOT) 2nd bot -14157
  // bottom of top hatch panel patch to bottom of ball holder frame 62mm
  public static final double ENCODER_LEVEL_THREE_CARGO = -19000 ;//-18784; // comp bot -18609; // 2nd bot -19019; // was 19144 (COMP BOT) 2nd bot -19467
  // bottom of high rocket cargo hole to bottom of ball holder frame 78mm 
  public static final double ENCODER_BEFORE_LEVEL_TWO_CLIMB = -4490; // test field -4133; // COMP BOT -3887;// 2nd bot -4565; // closest is low ball
  public static final double ENCODER_BEFORE_LEVEL_THREE_CLIMB = -15250;//-13526; // test field -13526;// COMP BOT-14601; // 2nd bot -15446; // closest is high ball
  public static final double ENCODER_DRIVE_WITH_CARGO = -12412.0; // comp bot unknown // 2nd bot -2195
  public static final double ELEVATOR_CLIMB_HOLD = 0.25; // to be tested
  public static final double ENCODER_HIGH_DRIVE_SLOW = -13000; //. Between level 2 cargo and level 3 hatch
  public static final double ELEVATOR_INIT_POWER = 0.2; //0.1
 
  // DeadBand
  public static final double DEADBAND_POSITIVE = 0.1;              // RIGHT JOYSTICK, RIGHT AND LEFT TRIGGERS
  public static final double DEADBAND_NEGATIVE = -0.1;             // RIGHT JOYSTICK 

  public static final double POST_CLIMB_DRIVE_TIME = 1.45; // WAS 1.15 tested well 13/3/19 WITH 2ND BOT
  public static final double POST_CLIMB_FINAL_DRIVE_TIME = 0.5; // tested well 13/3/19
  public static final int TIME_BEFORE_CLIMB = 40; //This would be 40
  public static final int kTimeoutMs = 0;
  
  // Intake Wheels
  public static final double CARGO_HOLD_POWER = -0.3;    // was 0.20
  public static final double CARGO_EJECT_POWER = 1.0; 
  public static final double CARGO_EJECT_DURATION = 0.5; 

  // Soleniod
  public static final double SOLENIOD_IN = 1.0;     //   Soleniod in = release hatch panel
  public static final double SOLENIOD_OUT = 0.0;    //   Soleniod out = hold hatch panel

  // Drive Train
  public static final double DRIVE_MAX_SPEED_Y = -1; //-0.6;     //was -0.75
  public static final double DRIVE_MAX_SPEED_Z = 0.25; // was 0.35
  public static final double DRIVE_SLOW_SPEED_Y = -0.3;
  public static final double DRIVE_SLOW_SPEED_Z = 0.25; // was 0.35
  public static final double QUICKTURN_THRESHOLD = 0.2;//2; //drive speed, under which, both wheels turn on the spot (like a tank)

  // Stop
  public static final double STOP_DOUBLE = 0.0;
  public static final int STOP_INT = 0;
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;
  //  
  
  
  /* 
	 * Global Variables 
	 */

  public static OI m_OI;	

	/* NavX */
	public static AHRS ahrs;
	
	/* Pigeon */
	public static PigeonIMU pigeon;
	public static final boolean usePigeonDefault = true; //If pigeon is broken, "false" her will revert to encoder difference for FollowArc

	/*
 	 * Constants
	 */


	public static final String DATADIR          = "/home/lvuser/motionprofiles/";
	public static final String POLL_WAYFILE_DIR = "/home/lvuser/pollwayfile/";  //This dir is Polled for our dynamicway.txt to generate a MP, then delete dynamicway.txt
	public static final String POLL_WAYFILE     = "dynamicway.txt";
	public static final String POLL_MP_DIR      = "/home/lvuser/pollmp/";       //We Poll this dir for generated MP dynamicmp.dat
	public static final String POLL_MPFILE      = "dynamicmp.dat";

	/* Vision Tracking - Limelight 2 */
	public static NetworkTable table;
	public static NetworkTableEntry tv;
	public static NetworkTableEntry tx;
	public static NetworkTableEntry ty;
	public static NetworkTableEntry ta;
	public static NetworkTableEntry camtran;

	public static final double inches2MetersFactor = 0.0254;
	public static final double inches2MMFactor = 25.4;
	public static final double mm2FeetFactor = 0.00328084;
	public static final double mm2MetersFactor = 0.001;
	
	public static final double WheelBase = 0.608; //0.605;  //distance between center of front wheels in meters
	public static final double EncoderPulsesToRotateBot360DegreesOnTheSpot = 35302; //17651; //17195;//16260; //13973.0; //15350;//11269.0;
	public static final double WheelRotationsPerMeter = 2.06185567; /* To get point.postion from MP converted to meters, ask how many rotations in 1 meter = 2.08865 theoretical, empircal from 10 rotation measurement: 2.06185567 */
	public static final double SensorUnitsPerRotation = 4096;
	//public static final double SensorUnitsPerMeter = 8343; //LHS measured 50059 pulses for 6m, so 8343.16667 for 1m 
    //public static final double SensorUnitsPerMeter = 8400;  //RHS measured 50397 pulses for 6m, so 8399.5 for 1m
	public static final double SensorUnitsPerMeter = 8740;  //dunno , just adjusted until 6m MotionProfile works????


	/*
	 * Vision Tracking
	 */
	//Camera centered and up high
	public static final double h1 = 943;  //Height(mm) of camera lens relative to ground 
	public static final double h2 = 800;  //Height (mm)
	public static final double a1 = 24.93;//12.9;//11.5;//20.3 good at 785mm, 22.2 good at 1340mm, 21.1;good at 1420mm   //Camera angle (degrees) of fixed lens relative to ground  
	public static       double a2 = 0; //Angle (degrees) of object as reported by Limelight2 relative to a1
	public static final double lensDistanceToBotFront = 75; //distance from camera to front bumper of bot
	public static final double lensDistanceToBotCenter = 405; //distance from camera to center of bot
	public static final double MaxVisionTrackMM = 5000; //For safety & ACCURACY (1.8M), limit vision tracking distance (in mm)
	public static final double MaxTrackingArea = 30; //110mm robot front lens from target
  public static final double MinTrackingArea = 0.7; //NEED to set this properly
  
/*  Camera down low Front Right
	public static final double h1 = 240;  //Height(mm) of camera lens relative to ground 
	public static final double h2 = 800;  //Height (mm)
	public static final double a1 = 12.0;//12.9;//11.5;//20.3 good at 785mm, 22.2 good at 1340mm, 21.1;good at 1420mm   //Camera angle (degrees) of fixed lens relative to ground  
	public static       double a2 = 0; //Angle (degrees) of object as reported by Limelight2 relative to a1
	public static final double lensDistanceToBotFront = 120; //distance from camera to front bumper of bot
	public static final double lensDistanceToBotCenter = 292; //distance from camera to center of bot
	public static final double MaxVisionTrackMM = 5000; //For safety & ACCURACY (1.8M), limit vision tracking distance (in mm)
	public static final double MaxTrackingArea = 4.4; //110mm robot front lens from target
	public static final double MinTrackingArea = 0.7; //NEED to set this properly
*/
	public static final double Pmin = 0.45; //Min Power % it takes to move robot in arcade (PercentVbus mode)
	public static final double Dss = 4000;  //Distance (mm) from target to Start Slowing in arcade
	public static final double LineUpTolerenceDegrees = 2; //noted that 1.6 degrees took RotateByAngle MP 4 trajectory segments (need min 5 to start profile)
	
	//public static 		double mptimer = Timer.getFPGATimestamp();
	

	
	/******* MOSTLY OLD STUFF SEE IF WE CAN DELETE ******/

	//public static DifferentialDrive RobotDriveBase;
	/*public static boolean teleopStarted=false;
	public static XboxController xboxBoi; 
	public static Encoder LEncode;
	public static Encoder REncode;
	public static RobotDriveBase Drive;
	public static Joystick m_leftstick;
	*/
	/*

  public static Trajectory.Config autonomousTrajectoryConfig;// =
			 //new Trajectory.Config(Trajectory.FitMethod.HERMITE_QUINTIC,//.HERMITE_CUBIC, 
					 //Trajectory.Config.SAMPLES_HIGH, 0.05, 2.0, 1.0, 30.0);
	public static Waypoint[] autonomousWayPoints;// = new Waypoint[] {
             //new Waypoint(0, 0, Pathfinder.d2r(90)),
             //new Waypoint(5, 5, 0)
             //new Waypoint(4, 8, Pathfinder.d2r(180)),
             //new Waypoint(0, 0, Pathfinder.d2r(90))
     //};
    public static Trajectory autonomousTrajectory;
    public static TankModifier autonomousTankTrajectoryModifier;
    public static Trajectory leftTrajectory;
    public static Trajectory rightTrajectory;    
	public static MotionProfileManager _mpFR;
	public static MotionProfileManager _mpFL;
	public static final double headingTolerance = 1; // what angle off target can we tolerate (at line-up & install hatch) 
  public static final double MaxDegreesForPIDSlot1 = 15; // Max Turn Angle to use Talon PID slot1, greater than this, use slot0
  */
	/*****^^^^ MOSTLY OLD STUFF ^^^^******/








}
