/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import edu.wpi.first.cameraserver.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX; 
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.Timer;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort.Port;



/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  MasterAutoSequencer mas;
  AutoSequenceSelector ass;
  ResetCommandGroup rcg;
  HatchPanelHold hph;

  public static SendableChooser<String> startingPosition = new SendableChooser<>();
  public static final String left_lvl_1 = "Left Level 1";
	public static final String right_lvl_1 = "Right Level 1";
	public static final String left_lvl_2 = "Left Level 2";
  public static final String right_lvl_2 = "Right Level 2";
  public static final String do_nothing = "Do Nothing";
	public static String m_startingPosition;
	
	public static SendableChooser<String> firstDestination = new SendableChooser<>();
	public static final String first_cargo_front = "First Cargo Front";
	public static final String first_cargo_near = "First Cargo Near 3";
	public static final String first_cargo_mid = "First Cargo Mid 2";
	public static final String first_cargo_far = "First Cargo Far 1";
	
	public static SendableChooser<String> secondDestination = new SendableChooser<>();
	public static final String second_cargo_cross_front= "Second Cargo Cross Front";
	public static final String second_cargo_near = "Second Cargo Near 3";
	public static final String second_cargo_mid = "Second Cargo Mid 2";
	public static final String second_cargo_far = "Second Cargo Far 1";
  
  // Motors       
  public static WPI_TalonSRX TalonFrontLeft;
  public static WPI_TalonSRX TalonFrontRight;
  public static WPI_TalonSRX TalonBackLeft;
  public static WPI_TalonSRX TalonBackRight;
  public static WPI_TalonSRX TalonElevator;
  public static WPI_TalonSRX TalonExtensionRail;
  public static WPI_TalonSRX TalonHabLift;
  public static WPI_VictorSPX SolenoidHatch;
  public static WPI_VictorSPX VictorIntake;
  public static WPI_VictorSPX VictorHabDrive;
  public static WPI_VictorSPX VictorDisableElevatorTop;

  // LED Control
  public static SpeedController blinkinLED;
  public Alliance colour;
  
  // Drive Group
  //public static DifferentialDrive drive;
  public static Drivetrain drivetrain;

  // Subsystems
  public static DriveTrainSubsystem driveTrainSub;

  public static IntakeWheels intakeWheelsSub;
  public static ExtensionRailSubsystem railSub;
  public static ElevatorLift elevatorSub;
  public static HabLift habLiftSub;
  public static HabDriveSubsystem habDriveSub;
  public static HatchPanelClamp hatchPanelClampSub;
  public static DisableElevatorTop disableElevatorTopSub;
  
  // Encoder
  public static Encoder elevatorEncoder;
  public static Encoder extensionRailEncoder;

  // Other  
  public static OI m_oi;

  // Timey Family
  public static Timer timeyClimbey;
  public static Timer timeyCargoEjecty;
  public static Timer timeyPostClimbDrive;
  public static Timer timeyDisableElevatorTop;
  public static Timer timeyDisableElevatorTopDelay;

  // Prepare Auto Trajectory Sequences
  public static FollowArc.FollowArcTrajectory[] trajectoriesToJoin = new FollowArc.FollowArcTrajectory[1];


  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
   
    // Camera
    // CameraServer.getInstance().startAutomaticCapture();
    
    // AutoSequenceSelector
    startingPosition.setDefaultOption("Do Nothing", do_nothing);
    startingPosition.addOption("Left Level 1", left_lvl_1);
    startingPosition.addOption("Left Level 2", left_lvl_2);
    startingPosition.addOption("Right Level 1", right_lvl_1);
    startingPosition.addOption("Right Level 2", right_lvl_2);
    
		SmartDashboard.putData("Robot position (Auto)", startingPosition);
    
   // firstDestination.addOption("First Cargo Near 3", first_cargo_near);
		firstDestination.setDefaultOption("First Cargo Front", first_cargo_front);
		//firstDestination.addOption("First Cargo Mid 2", first_cargo_mid);
		//firstDestination.addOption("First Cargo Far 1", first_cargo_far);
		SmartDashboard.putData("First Destination", firstDestination);
		
		//secondDestination.addOption("Second Cargo Cross Front", second_cargo_cross_front);
		secondDestination.setDefaultOption("Second Cargo Near 3", second_cargo_near);
		//secondDestination.addOption("Second Cargo Mid 2", second_cargo_mid);
		//secondDestination.addOption("Second Cargo Far 1", second_cargo_far);
	//	SmartDashboard.putData("Second Destination", secondDestination);
 
    TalonExtensionRail = new WPI_TalonSRX(RobotMap.TalonExtensionRail);
      TalonExtensionRail.configFactoryDefault();
    TalonHabLift = new WPI_TalonSRX(RobotMap.TalonHabLift);
      TalonHabLift.configFactoryDefault();
    TalonFrontLeft = new WPI_TalonSRX(RobotMap.TalonFrontLeft);
      TalonFrontLeft.configFactoryDefault();
    TalonFrontRight = new WPI_TalonSRX(RobotMap.TalonFrontRight);
      TalonFrontRight.configFactoryDefault();
    TalonBackLeft = new WPI_TalonSRX(RobotMap.TalonBackLeft);
      TalonBackLeft.configFactoryDefault();
    TalonBackRight = new WPI_TalonSRX(RobotMap.TalonBackRight);
      TalonBackRight.configFactoryDefault();
    TalonElevator = new WPI_TalonSRX(RobotMap.TalonElevator);
      TalonElevator.configFactoryDefault();
    VictorIntake = new WPI_VictorSPX(RobotMap.VictorIntake);
    VictorHabDrive = new WPI_VictorSPX(RobotMap.VictorHabDrive);
    SolenoidHatch = new WPI_VictorSPX(RobotMap.VictorSolenoidHatch);
    VictorDisableElevatorTop = new WPI_VictorSPX(RobotMap.VictorDisableElevatorTop);
    //blinkinLED = new Spark(RobotMap.SparkLED);
    VictorIntake.setInverted(true);
    
    TalonElevator.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    
    TalonElevator.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    TalonHabLift.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    TalonHabLift.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    TalonExtensionRail.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    TalonExtensionRail.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);

    // Set PIDs
    TalonElevator.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
    TalonElevator.config_kP(0, RobotMap.ELEVATOR_PID_P, RobotMap.kTimeoutMs);
    TalonElevator.config_kI(0, RobotMap.ELEVATOR_PID_I, RobotMap.kTimeoutMs);
    TalonElevator.config_kD(0, RobotMap.ELEVATOR_PID_D, RobotMap.kTimeoutMs);  
    TalonElevator.configPeakOutputForward(RobotMap.ELEVATOR_MAX_FORWARD_SPEED);
    TalonElevator.configPeakOutputReverse(RobotMap.ELEVATOR_MAX_REVERSE_SPEED);
    TalonElevator.setSensorPhase(true);
    TalonElevator.setInverted(true);
    

    TalonExtensionRail.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0 ,0);
    TalonExtensionRail.config_kP(0, RobotMap.EXTENSION_RAIL_PID_P, RobotMap.kTimeoutMs);
    TalonExtensionRail.config_kI(0, RobotMap.EXTENSION_RAIL_PID_I, RobotMap.kTimeoutMs);
    TalonExtensionRail.config_kD(0, RobotMap.EXTENSION_RAIL_PID_D, RobotMap.kTimeoutMs); 
    TalonExtensionRail.configPeakOutputForward(RobotMap.EXTENSION_RAIL_MAX_FORWARD_SPEED);
    TalonExtensionRail.configPeakOutputReverse(RobotMap.EXTENSION_RAIL_MAX_REVERSE_SPEED); 
    TalonExtensionRail.setSensorPhase(false);
    TalonExtensionRail.setInverted(false);
    
    TalonHabLift.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0 ,0);
    TalonHabLift.config_kP(0, RobotMap.HABLIFT_PID_P, RobotMap.kTimeoutMs);
    TalonHabLift.config_kI(0, RobotMap.HABLIFT_PID_I, RobotMap.kTimeoutMs);
    TalonHabLift.config_kD(0, RobotMap.HABLIFT_PID_D, RobotMap.kTimeoutMs);
    TalonHabLift.configPeakOutputForward(RobotMap.HABLIFT_MAX_FORWARD_SPEED);
    TalonHabLift.configPeakOutputReverse(RobotMap.HABLIFT_MAX_REVERSE_SPEED);
    TalonHabLift.setSensorPhase(true);
    TalonHabLift.setInverted(true);
    
    // Setup Auto MP Trajectories   
/*    trajectoriesToJoin[0] = new FollowArc.FollowArcTrajectory();
    trajectoriesToJoin[0].trajectoryFilename = "/home/lvuser/motionprofiles/RightLevel1ToRightCargoFront.dat";
    trajectoriesToJoin[0].td = 60;
    trajectoriesToJoin[0].usePigeonIMU = true;
    trajectoriesToJoin[0].inReverse = false;
  */ 
   

    // FollowerMode
    TalonBackLeft.follow(TalonFrontLeft);
    TalonBackRight.follow(TalonFrontRight);

    // Subsystems
    //driveTrainSub = new DriveTrainSubsystem();
    drivetrain = new Drivetrain();
    intakeWheelsSub = new IntakeWheels();
    railSub = new ExtensionRailSubsystem();
    elevatorSub = new ElevatorLift();
    habLiftSub = new HabLift();
    habDriveSub = new HabDriveSubsystem();
    hatchPanelClampSub = new HatchPanelClamp();
    disableElevatorTopSub = new DisableElevatorTop();
  
    // Operator Interface
    m_oi = new OI();

    // Motor Drive Groups
    //MDE20190310 drive = new DifferentialDrive(Robot.TalonFrontLeft,Robot.TalonFrontRight);
    //MDE20190310 drive.setSafetyEnabled(true); // Recommended for drivetrains
    
    // Timey Family
    timeyCargoEjecty = new Timer();
    timeyClimbey = new Timer();
    timeyPostClimbDrive = new Timer();
    timeyDisableElevatorTop = new Timer();
    timeyDisableElevatorTopDelay = new Timer();
   
    // NavX (Gyro)
		try {
      RobotMap.ahrs = new AHRS(Port.kUSB /*SPI.Port.kMXP*/);
      RobotMap.ahrs.reset();  /******* Do this initialize at start of game on the Hub */
    } catch (RuntimeException ex) {
      System.out.println("Error intializing navX-MXP: " + ex.getMessage());
    } 

    // Pigeon IMU (Gyro)
    drivetrain.resetPigeon();


    // LED
    colour = DriverStation.getInstance().getAlliance();
    /*
    if(colour == DriverStation.Alliance.Blue){
      blinkinLED.set(0.87);                                                     
    }
    if(colour == DriverStation.Alliance.Red){
      blinkinLED.set(0.61);  
    }
    if(colour == DriverStation.Alliance.Invalid){
      blinkinLED.set(0.69); 
    }
    blinkinLED.setInverted(false); */ // LEDs not yet on robot
    mas = new MasterAutoSequencer();
    mas.addSequential(new ResetCommandGroup());
    mas.addSequential(new HatchPanelHold());
    mas.addSequential(new AutoSequenceSelector());
    
    /*
    rcg = new ResetCommandGroup();
    ass = new AutoSequenceSelector();
    hph = new HatchPanelHold();*/

  }
  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Elevator Encoder", TalonElevator.getSelectedSensorPosition());
    SmartDashboard.putNumber("ExtensionRail Encoder", TalonExtensionRail.getSelectedSensorPosition());
    SmartDashboard.putNumber("HabLift Encoder", TalonHabLift.getSelectedSensorPosition());
    SmartDashboard.putBoolean("Elevator Bottom Limit Switch", TalonElevator.getSensorCollection().isFwdLimitSwitchClosed());
    SmartDashboard.putBoolean("Elevator Top Limit Switch", TalonElevator.getSensorCollection().isRevLimitSwitchClosed());
    SmartDashboard.putBoolean("HAB Retracted Limit Switch", TalonHabLift.getSensorCollection().isFwdLimitSwitchClosed());
    SmartDashboard.putBoolean("HAB Extended Limit Switch", TalonHabLift.getSensorCollection().isRevLimitSwitchClosed());
    SmartDashboard.putBoolean("ExtensionRail Retracted Limit Switch", TalonExtensionRail.getSensorCollection().isFwdLimitSwitchClosed());
    SmartDashboard.putBoolean("ExtensionRail Extended Limit Switch", TalonExtensionRail.getSensorCollection().isRevLimitSwitchClosed());
    
  }
  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
    
  }

  @Override
  public void disabledPeriodic() {

    Scheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    // new ResetHabEncoder().start();   
    // new ResetElevatorEncoder().start();
    // new ExtensionRailResetEncoderStayIn().start();
    //   new ResetExtensionRailEncoder().start();
     //new ResetCommandGroup().start();
     
     //if (rsg != null)
     //  rsg.start();
     
       //new AutoSequenceSelector();
     //if (ass != null)
      // ass.start(); // commented out during testing
    // new TestCommandGroup().start(); // commented out during testing
     //new HatchPanelHold().start();
     
     /*if (hph != null)
       hph.start();
     */  
    /*mas.addSequential(rcg);
    mas.addSequential(ass);
    mas.addSequential(hph);*/
    mas.start();
    
    
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
  
    // new TestCommand().start();
    // new ResetElevatorEncoder().start();
    // new ResetExtensionRailEncoder().start();
   // new HabClimbRetract().start();   
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    /*if (m_autonomousCommand != nu ll) {
      m_autonomousCommand.cancel(); 
    }
    */  
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
   // SmartDashboard.putNumber("Extension Rail SetPoint", TalonExtensionRail.getClosedLoopTarget());
    Scheduler.getInstance().run();
        //  new TestCommand().start();
       // new ResetHabEncoder().start();
    }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
   
    //new ResetCommandGroup().start();
    // new ResetExtensionRailEncoder().start();
    //  new ResetElevatorEncoder().start();              
   // new HabResetEncoder().start();                   
   // new ExtensionRailResetEncoderStayIn().start();
   
  }
}
