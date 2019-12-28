package frc.robot.subsystems;

import frc.robot.RobotMap;
import frc.models.BobTalonSRX;
import frc.models.DriveSignal;
import frc.models.LeaderBobTalonSRX;
import frc.models.SRXGains;
import frc.robot.commands.drivetrain.BobDrive;
//import frc.robot.commands.DriveTrainCommand;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Drivetrain extends Subsystem {
	public boolean habAutoDrive  = false;
	public boolean isElevatorHigh = false; 
	public boolean isClosedLoopFailure = false;
  
	
	
	private int counter=0;
	
	private boolean usePigeon=RobotMap.usePigeonDefault;
	private boolean isHighGear = false;


	public static int LOW_GEAR_PROFILE = 2;
	public static int HIGH_GEAR_PROFILE = 0;
	public static int ROTATION_PROFILE = 1;
/*	
	public double rotationP = 1.8;
	public double rotationI = 0.001;
	public double rotationD = 50.0; 
	public double rotationF = 0.0;
	public int rotationIZone = 30; 
*/
/* WORKS PRETTY WELL FOR LOADING TO ROCKET NEAR (2 move) although not always repeatable
*  at td=60, not good at td=30
*/
/*public double rotationP = 0.5605; //0.65 good
	public double rotationI = 0.0003;//0.0001
	public double rotationD = 10; //10
	public double rotationF = 0;
	public int rotationIZone = 30;*/
/* try increasing p for td=30 */

/**** SAVING THIS 
public double rotationP = 0.783; //0.65 good
	public double rotationI = 0.0001;//0.0001
	public double rotationD = 80; //10
	public double rotationF = 0;
	public int rotationIZone = 30;
****/

/* Been using this one */
    public double rotationP = 0.783; //0.913; //0.65 good
	public double rotationI = 0.0001;//0.0001
	public double rotationD = 50; //10
	public double rotationF = 0;
	public int rotationIZone = 30;



	/*conservative 
	
	public double rotationP = 0.65;
	public double rotationI = 0.0;
	public double rotationD = 1; 
	public double rotationF = 0;
	public int rotationIZone = 30;
*/
/* good
public double rotationP = 0.8;
	public double rotationI = 0.0;
	public double rotationD = 4; 
	public double rotationF = 0;
	public int rotationIZone = 30;
	*/ 
/*	public double rotationP = 1.0;
	public double rotationI = 0.0005;
	public double rotationD = 10; 
	public double rotationF = 0;
	public int rotationIZone = 30; 
*/
	
	// Mag Encoder Gains
	private SRXGains lowGearGains = new SRXGains(LOW_GEAR_PROFILE, 0.600, 0.0, 12.00, 0.0763, 0);
	//good private SRXGains highGearGains = new SRXGains(HIGH_GEAR_PROFILE, 0.1500	, 0.0015, 0.0, 0.30785, 10);
	//private SRXGains highGearGains = new SRXGains(HIGH_GEAR_PROFILE, 0.15, 0.0, 0.0, 0.30785, 50);
	//working private SRXGains highGearGains = new SRXGains(HIGH_GEAR_PROFILE, 0.15, 0.0015, 12.0, 0.30785, 50);
	//** */private SRXGains highGearGains = new SRXGains(HIGH_GEAR_PROFILE, 2.45, 0.0, 0.0, 0.30785, 50);
	private SRXGains highGearGains = new SRXGains(HIGH_GEAR_PROFILE, 0.15, 0.0015, 12.0, 0.30785, 50);
	//private SRXGains highGearGains = new SRXGains(HIGH_GEAR_PROFILE, 1.0, 0.0, 0.0, 0, 50);
	

    //private SRXGains highGearGains = new SRXGains(HIGH_GEAR_PROFILE, 0.25, 0.0, 0.0, 0.05115, 50);
	
    private SRXGains rotationGains = new SRXGains(ROTATION_PROFILE, rotationP, rotationI, rotationD, rotationF, rotationIZone); 
	
	private BobTalonSRX leftFollower;
	private BobTalonSRX rightFollower;
	public LeaderBobTalonSRX leftLead; 
	public LeaderBobTalonSRX rightLead;
	private PigeonIMU pigeon;
	//DifferentialDrive diffDrive;
	
	public Drivetrain() {
		counter=0;

		/* Lets see what happens when we set up a Differencetial Drive, then do our MP setups afterward 
		WPI_TalonSRX leftDiff = new WPI_TalonSRX(RobotMap.TalonFrontLeft);
		WPI_TalonSRX rightDiff = new WPI_TalonSRX(RobotMap.TalonFrontRight);


		diffDrive = new DifferentialDrive(leftDiff, rightDiff);
*/
		/* Now, also wrap our Talons in a way suitable for MP usage */ 
		setupDrivetrain();

	}

	public void setupDrivetrain() {
	System.out.println("SETTING UP DRIVETRAIN...");
    	leftFollower = new BobTalonSRX(RobotMap.TalonBackLeft);
    	leftFollower.configFactoryDefault();
    	rightFollower = new BobTalonSRX(RobotMap.TalonBackRight);
    	rightFollower.configFactoryDefault();   
    	leftLead = new LeaderBobTalonSRX(RobotMap.TalonFrontLeft, leftFollower); 
    	leftLead.configFactoryDefault();
    	rightLead = new LeaderBobTalonSRX(RobotMap.TalonFrontRight, rightFollower);
		rightLead.configFactoryDefault();

	
		//If you change the pigeon to another follower, also change setNavModePigeonAngle()
    	pigeon = new PigeonIMU(rightFollower);
    	rightLead.clearMotionProfileTrajectories();
    	
    	// These Values will be different for every Robot :)
		leftLead.setInverted(false);
		leftLead.configPrimaryFeedbackDevice(FeedbackDevice.CTRE_MagEncoder_Relative);
		leftLead.setSensorPhase(true);

		rightLead.setInverted(true);
		rightLead.configPrimaryFeedbackDevice(FeedbackDevice.CTRE_MagEncoder_Relative);
		rightLead.setSensorPhase(true);
			
		leftLead.enableCurrentLimit(false);
		leftLead.configContinuousCurrentLimit(60);
		rightLead.enableCurrentLimit(false);
		rightLead.configContinuousCurrentLimit(60);

		//time required to ramp up to speed 
		leftLead.configOpenloopRamp(0.25);
		rightLead.configOpenloopRamp(0.25);
		rightLead.configClosedLoopPeriod(0,5);
		rightLead.configClosedLoopPeriod(1,5);
		

	
		rightLead.configNominalOutputForward(0.04);
		rightLead.configNominalOutputReverse(-0.075);
		rightLead.configPeakOutputForward(1);
		rightLead.configPeakOutputReverse(-1);

		leftLead.configNominalOutputForward(0.04);
		leftLead.configNominalOutputReverse(-0.075);
		leftLead.configPeakOutputForward(1);
		leftLead.configPeakOutputReverse(-1);


		setNeutralMode(NeutralMode.Brake/*Coast*/);

		//configs PID Gains 
		configGains(highGearGains);
		configGains(lowGearGains);
		configGains(rotationGains);

        /* 
         * Right Side is "Master" ie is running the closed loops (PIDF[0] and PIDF[1])
         * Left Side is the "Auxillary Follower" 
         * When the Left site is not inverted:
         *     Right-motor-output = PIDF[0] + PIDF[1] 
         *     Left-motor-output  = PIDF[0] - PIDF[1]
         * When target distance is forward PIDF[0] is positive on both left and right
         * When target heading is left PIDF[1] is positive causing positive on right and negative on left (turn left)
         * Can use encode difference for heading (right encoder minus left encoder) gives increasing signal as robot turns left.
         * Can use Pigeon IMU for heading, it gives increasing signal when robot turns left. 
    	 * configure distance sensor
		 * Remote 0 will be the other side's Encoder
	     */
	
		rightLead.configRemoteSensor0(leftLead.getDeviceID(), RemoteSensorSource.TalonSRX_SelectedSensor);
		/* 
		 * SensorSum        = Sum0 + Sum1
		 * SensorDifference = Diff0 - Diff1
		 * Our leftLead is more negative going forwards
		 * Our rightLead is more positive going forwards
		 * We want to end up with +right - -left = right + left
		 *      +rightQuadEncoder - -leftRemoteSensor0 = rightQuadEncoder + leftRemoteSensor0
         * 
         * place RemoteSensor0 in Sensor Term Sum0 and our local QuadEncoder in Sum1
		 */
		rightLead.configSensorSum(FeedbackDevice.CTRE_MagEncoder_Relative, FeedbackDevice.RemoteSensor0);
		
		
		/*
		 * The following should be SensorSum if both encoder values are positive when pushed forward
		 *       or FeedbackDevice.SensorDifference if one encoder is negative when pushed forward
		 *                     
		 *          The aim is to add the absolute values and divide by 2 to get the average encoder value.            
		 *
		 * NB: SensorSum        = Sum0 + Sum1
		 *     SensorDifference = Diff0 - Diff1   (ie need to load Diff0, Diff1 in a sensible order) 
		 *
		 */
		//rightLead.configPrimaryFeedbackDevice(FeedbackDevice.SensorDifference, 0.5); //fwd: one encoder is pos, other neg
		rightLead.configPrimaryFeedbackDevice(FeedbackDevice.SensorSum, 0.5); //fwd: both encoders are positive
																		
		
		//I is limited to a certain amount
		//rightLead.configMaxIntegralAccumulator(ROTATION_PROFILE, 3000);

		// configure angle sensor
		// Remote 1 will be a pigeon		
  		//rightLead.configRemoteSensor1(leftFollower.getDeviceID(), RemoteSensorSource.GadgeteerPigeon_Yaw); 
		//rightLead.configRemoteSensor1(rightFollower.getDeviceID(), RemoteSensorSource.GadgeteerPigeon_Yaw); 
	    //rightLead.configSecondaryFeedbackDevice(FeedbackDevice.RemoteSensor1, (3600.0 / 8192.0)); // Coefficient for convert to 360
		//usePigeonAuxPos(); //config to use pigeon by default 
		if (usePigeon)
			setNavModePigeonAngle();
		else
			setNavModeEncoderDiffAngle(); //config to use encoderDiff  by default 
		
		leftLead.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, 0);
		rightLead.configAuxPIDPolarity(false, 0); //false: primary = PID0 - PID1, aux = PID0 + PID1
		                                          //true:  primary = PID0 + PID1, aux = PID0 - PID1
    }
	
	
	public void initDefaultCommand() {
		setDefaultCommand(new BobDrive());
		//setDefaultCommand(new DriveTrainCommand());
	}

	public void configGains(SRXGains gains) {
		this.leftLead.setGains(gains);
		this.rightLead.setGains(gains);
	}

	
	//left and right are the move values passed into the method from the driver sticks
	public void drive(ControlMode controlMode, double left, double right) {
		this.leftLead.set(controlMode, left);
		this.rightLead.set(controlMode, right);
	}

	    
	/*public void curvatureDrive(double y, double z){
		 *
         * Hmmm is this difference becaue of the physical wiring of the motors?
		 *
		diffDrive.curvatureDrive(z, -y, true);
	} 
    */
    
	
	public void stopDrivetrain() {
		this.leftLead.set(ControlMode.PercentOutput, 0);
		this.rightLead.set(ControlMode.PercentOutput, 0);
	}

	public void drive(ControlMode controlMode, DriveSignal driveSignal) {
		this.drive(controlMode, driveSignal.getLeft(), driveSignal.getRight());
	}

	public void setCrossTheFieldRotatePValue() {
		rotationP = 1.6;
	}

	public void setDefaultRotationSRXGains() {
		rotationP = 1.8;
	}

	public double getLeftDriveLeadDistance() {
		return this.leftLead.getSelectedSensorPosition();
	}

	public double getRightDriveLeadDistance() {
		return this.rightLead.getSelectedSensorPosition();
	}

	public double getLeftDriveLeadVelocity() {
		return this.leftLead.getSelectedSensorVelocity();
	}

	public double getRightDriveLeadVelocity() {
		return this.rightLead.getSelectedSensorVelocity();
	}

	public void setDrivetrainPositionToZero() {
		this.leftLead.setSelectedSensorPosition(0);
		this.rightLead.setSelectedSensorPosition(0);
	}

	public double getLeftLeadVoltage() {
		return this.leftLead.getMotorOutputVoltage();
	}

	public double getRightLeadVoltage() {
		return this.rightLead.getMotorOutputVoltage();
	}

	public double getLeftClosedLoopError() {
		return this.leftLead.getClosedLoopError();
	}

	public double getRightClosedLoopError() {
		return this.rightLead.getClosedLoopError();
	}

	public TalonSRX getLeftLeadTalon() {
		return this.getLeftLeadTalon();
	}

	public TalonSRX getRightLeadTalon() {
		return this.rightLead;
	}

	public void setNeutralMode(NeutralMode neutralMode) {
		this.leftLead.setNeutralMode(neutralMode);
		this.rightLead.setNeutralMode(neutralMode);
	}

	public boolean isHighGear() {
		return isHighGear;
	}

	public void setIsHighGear(boolean isHighGear) {
		this.isHighGear = isHighGear;
		//setting class variable equal to method variable
	}

	public double getNavAngle() {
		double angle=0;
		if (usePigeon) {
		    double[] ypr = new double[3];
		    pigeon.getYawPitchRoll(ypr);
			//return ypr[0]/64;
			
			/*PigeonIMU.GeneralStatus gs = new PigeonIMU.GeneralStatus();
			pigeon.getGeneralStatus(gs);
			System.out.printf("RAW YAW: %s div:%s pigeonState:%s mode:%s \r", ypr[0], ypr[0]*360/8192, gs.state,gs.currentMode );
			*/
			angle =  ypr[0];
		} else {
			angle = 2*(((getRightDriveLeadDistance() - getLeftDriveLeadDistance()) * 360.0)/RobotMap.EncoderPulsesToRotateBot360DegreesOnTheSpot);
			SmartDashboard.putNumber("Calculated Angle", angle);
			angle = rightLead.getSecondarySensorPosition()/10.0;
			SmartDashboard.putNumber("Secondary Sensor Angle", angle);
			angle = 2*(((rightLead.getSensorCollection().getQuadraturePosition() - leftLead.getSelectedSensorPosition()) * 360.0)/RobotMap.EncoderPulsesToRotateBot360DegreesOnTheSpot);
			SmartDashboard.putNumber("Correct Calculated Angle", angle);
		}
		return angle;
	}

	/*
	 * Get the equivalent angle withing the range: 0 - 360 degrees
	 */
	public double bound360(double angle){
		double angle360;
		angle360 = angle % 360; //get remaind when divided by 360 (this can be neg.)
		angle360 = (angle360 + 360) % 360; //force it to be a positive ie 0 <= angle < 360
		return angle360;
	}

	public void setNavAngle(double degrees) {
		
		System.out.println("setAngle Running");
		if (this.usePigeon) {
			SmartDashboard.putNumber("Pigeon Angle Set - Sensors", degrees);
			pigeon.setYaw(degrees);
		}
		else {	
            /* Can't just set Sensor, have to to set selected Sensor on the left and raw encoder on the right (master) */
			rightLead.getSensorCollection().setQuadraturePosition((int)Math.round((degrees/360)*RobotMap.EncoderPulsesToRotateBot360DegreesOnTheSpot), 0);
			leftLead.setSelectedSensorPosition(0);
			//leftLead.getSensorCollection().setQuadraturePosition(0, 0);
			
		    /* 
		     * Need to set one backwards by half degrees_pulses and one forwards by half degresses_pulses so that
		     * the difference never exceeds  
		     */
			
			}
	}
	
	public double getDistance() {
		return rightLead.getPrimarySensorPosition();
	}

	public double getVelocity() {
		return rightLead.getPrimarySensorVelocity();
	}

	public void setNavModePigeonAngle() {
		//NB: If you move the Pigeon to the other follower, also change the config in setupDrivetrain()
		rightLead.configRemoteSensor1(rightFollower.getDeviceID(), RemoteSensorSource.GadgeteerPigeon_Yaw); 
		rightLead.configSecondaryFeedbackDevice(FeedbackDevice.RemoteSensor1, (3600.0 / 8192.0)); // Coefficient for
	}
	
	
    /*
	 * setNavModeEncoderDiffAngle - Mode to use encoders to determine heading...
	 *
	 */	
	public void setNavModeEncoderDiffAngle() {
	    //rightLead.configRemoteSensor1(leftLead.getDeviceID(), RemoteSensorSource.TalonSRX_SelectedSensor);
	    //rightLead.configSensorSum(FeedbackDevice.RemoteSensor1, FeedbackDevice.QuadEncoder);
	    /* 
         * SensorSum = Sum0 + Sum1
         *            -leftEncoder + +rightEncoder 
	     */
	    //TEMP RM rightLead.configSecondaryFeedbackDevice(FeedbackDevice.SensorDifference, 0.5); 
		//rightLead.configRemoteSensor1(leftLead.getDeviceID(), RemoteSensorSource.TalonSRX_SelectedSensor);
	    //assume this is done: rightLead.configRemoteSensor0(leftLead.getDeviceID(), RemoteSensorSource.TalonSRX_SelectedSensor);
		/*
	     * Coefficient Calc
	     *   - Feedback is compared against desired MotionProfile points heading (eg 120degrees x 10 = 1200)
	     *   -if left is -5000 and right is +5000 then we are facing straight ahead as sum = 
	     *   -if left is =-5100 and right is +5000 then we are facing, sum = Xs = 100, so Zd degrees to the left
	     *   -a single wheel travelling in a complete bot-on-spot (opposite wheel turning in opposit direction)
	     *     - circle 360degrees 2x(pi)xr = diam. x (pi) = wheelbase x (pi) = 605mm x (pi) = 1900.6mm
         *     - single wheel does 3.9698 rotations
         *     - quad encoders do 3007.656 pulses per rotation
         *     - therefore it takes 11939.809 pulses of each encoder for robot to do 360 turn on the spot
	     *   -so Zd = Xs
	     *   -left turn: -leftEnc more toward positive and +rightEnc goes more positive
	     *           eg   -4950                 and +5050   sum = 100    encoder change +50 to each wheel
	     *   - so generally speaking, for 360 turn left, we would apply +11940 pulses to each wheel
	     *   - that was on paper, but real world 10 turn right test says SUM= -112693 pulses for 10 x 360 degrees right
	     *   - since we are dealing with a fraction of 3600 (10 x 360 degrees loaded into MP auxPosition)
	     *       - 360 right turn SUM = -11269
	     *       - as a fraction of 3600 (MP heading loaded into auxPosition)         
	     *       - Coefficient = (SUM x 3600)/11296    
	     *   
	     *   
	     *    if we just want correction to apply to each wheel 
	     *        we would need XXs/2
	     */
		//rightLead.configSensorSum( FeedbackDevice.QuadEncoder,FeedbackDevice.RemoteSensor0);	    
		//rightLead.configRemoteSensor0(leftLead.getDeviceID(), RemoteSensorSource.TalonSRX_SelectedSensor);
		//rightLead.confisgSecondaryFeedbackDevice(FeedbackDevice.SensorSum,3600.0/11269.0); //on paper 360 is 11940 pulses, 10 turn test says 11269
		//rightLead.configAuxPIDPolarity(false, 0);
		
		//FYI: in init ==> rightLead.configRemoteSensor0(leftLead.getDeviceID(), RemoteSensorSource.TalonSRX_SelectedSensor);
		
		//Loadup Diff0 and Diff1  (in this case it's rightencoder - leftselectedencoder)
		//SensorDifference = Diff0 - Diff1
		rightLead.configSensorDiff( FeedbackDevice.CTRE_MagEncoder_Relative,FeedbackDevice.RemoteSensor0);
		
		//Coeeficient = SensorDiff result pulses as portion of pulses for 360 degress turn on the spot
		//             3600 is used as auxiliary input to represent 360 degrees
		rightLead.configSecondaryFeedbackDevice(FeedbackDevice.SensorDifference,3600.0/RobotMap.EncoderPulsesToRotateBot360DegreesOnTheSpot); //on paper 360 is 11940 pulses, 10 turn test says 11269
	}

	public void resetPosition() {
	  rightLead.setSelectedSensorPosition(0);
      leftLead.setSelectedSensorPosition(0);
      
 	}
	
	public void resetPigeon() {
		//this.pigeon.clearStickyFaults();
		this.pigeon.configFactoryDefault();
		//Yaw is rotation of robot during autos 
		this.pigeon.setYaw(90.0);//, 0);
		this.pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR, 5);
		this.pigeon.setAccumZAngle(0);
	}

	@Override
	public void periodic() {
		counter++;
		if ((counter % 25)==0) { 
			SmartDashboard.putBoolean("Drivetrain High Gear", isHighGear);
			SmartDashboard.putNumber("Left Drive Position", getLeftDriveLeadDistance());
			SmartDashboard.putNumber("Right Drive Position", getRightDriveLeadDistance());

			SmartDashboard.putNumber("***Left Encoder Value", leftLead.getSensorCollection().getQuadraturePosition());
			SmartDashboard.putNumber("***Right Encoder Value", rightLead.getSensorCollection().getQuadraturePosition());
			
			SmartDashboard.putNumber("***Drivetrain Angle (getNavAngle())", getNavAngle());
			SmartDashboard.putNumber("***NavX Angle", RobotMap.ahrs.getAngle());
			
			SmartDashboard.putNumber("Drivetrain Velocity", getVelocity());
			SmartDashboard.putNumber("Drivetrain Distance", getDistance());
			SmartDashboard.putNumber("Left Lead Current", leftLead.getOutputCurrent());
			SmartDashboard.putNumber("Left Follower Current", leftFollower.getOutputCurrent());
			SmartDashboard.putNumber("Right Lead Current", rightLead.getOutputCurrent());
			SmartDashboard.putNumber("Right Follower Current", rightFollower.getOutputCurrent());
		}	
	}
}

