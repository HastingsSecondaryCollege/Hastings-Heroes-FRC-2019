  /*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StickyFaults;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;
import frc.models.BobTalonSRX;
import frc.utils.BobDriveHelper;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.Notifier;


public class LimeLightTrackPIDToAreaLS extends Command {

  // These numbers must be tuned for your Robot!  Be careful!


  /*
    * (DESIRED_TARGET_AREA - ta)  x  DRIVE_K 
    * 
    * DRIVE_K is a factor to get (DESIRED_TARGET_AREA - ta) to an effective dimishing
    * drive speed as we approach target.  As soon as we hit DESIRED_TARGET, the speed drops off.
    * NB: Any speed above the MAX_DRIVE (or SLOW_DRIVE) will be clipped and we lose that part of the 
    * range for which we wil deaccellerate.
    *  
    */                                                
  final double DRIVE_K = 0.66;//0.26;                    // how hard to drive fwd toward the target
  final double SLOW_DRIVE_K = 0.3; //0.3                // used if starting from close at slow speed
  //final double TX_OFFSET = 0;//4.5; // If limelight tx value is incorrect, calibrate in Output -> Crosshair A -> X only in limelight browser
  //Error Angle offset when bot is misaligned
                                /* Manual sight lens on center of target, click 'Only X' button in Output -> Crosshair A

  /*
    * Max Drive Speeds (0 stopped, 1 is full 100% PercentOut drive) as we track to target
    */                                                
  final double MAX_DRIVE = 0.6; //0.4                   // Simple speed limit so we don't drive too fast
  final double SLOW_DRIVE = 0.3; //0.19;   

  final double SLOW_VELOCITY = 700; // 500; //14;

  final double RESET_LAST_TX_VALUE = 9999.123;
  final double RESET_LAST_TA_VALUE = 0.0;
  final double ACCEPTABLE_TX_DELTA = 13;

  /*
    * Target Area - vision target areas detected for which we start to slow down rapidly
    *
    * Need to account for current Speed and Overshoot ie cutoff at area of 7 to stop at area of 29 
    * NB: If you are going slower you can start cutoff closer to final area (not much overshoot).

    *   AUTON  - assumes driving at MAX_DRIVE and slowing using DRIVE_K factor
    *   TELEOP - assumes driving at MAX_DRIVE and slowing using DRIVE_K factor
    *   SLOW   - assumes driving at SLOW_DRIVE and slowing using SLOW_DRIVE_K factor
    */

  /* 2019 Bot B */
  final double DESIRED_TARGET_AREA_SLOW   = 20.0; //25.0; //29.0; //% Area* of the target when the robot reaches the wall
  final double DESIRED_TARGET_AREA_AUTON  = 7.1; //6.8; //10.60; //29.0; //% Area* of the target when the robot reaches the wall
  final double DESIRED_TARGET_AREA_TELEOP = 2.0; //2.8; //3.875; //5.11;//7.5; //29.0; //% Area* of the target when the robot reaches the wall
  final double DESIRED_TARGET_AREA_SLOW_TELEOP = 7.5; // 9.0; //17.3;  //aera that represents 18.5cm away (no momentum) ie when approaching slowly
      
  /* 2018 Bot B *
  final double DESIRED_TARGET_AREA_SLOW   = 25.0; //29.0; //% Area of the target when the robot reaches the wall
  final double DESIRED_TARGET_AREA_AUTON  = 14.60; //29.0; //% Area of the target when the robot reaches the wall
  final double DESIRED_TARGET_AREA_TELEOP = 7.5; //29.0; //% Area of the target when the robot reaches the wall   
  */
  final double SAMPLE_ZONE_AREA = 2.0; // 10.0;// check this num  ber, we start making frequent correction when we reach this are
  final double SLOW_TARGET_APPROACH_ZONE_AREA = 1.0; //2.0; //5.0; //Do slow approach if starting withion this zone 

/** MAIN TUNING PARAMS - Related to Momentum/Weight of Bot
 * 
 * MAX_DRIVE - BEWARE: Changing this effects EVERYTHING
 * To avoid Dead Zone - SLOW_TARGET_APPROACH_ZONE_AREA needs to be around 0.875 less than DESIRED_TARGET_AREA_TELEOP
 *                      NB: if these values are the same you get a dead zone.  
 *                          if SLOW_TARGET_APPROACH_ZONE_AREA is larger, you get a dead zone
 * DESIRED_TARGET_AREA_SLOW_TELEOP - is the target area, including the small momentum when driving at 
 *                          SLOW_DRIVE and applying (DESIRED_TARGET_AREA_SLOW_TELEOP - ta) deacceleration 
 *                      ie in a test of limebot, 17 is the area when you pull up with zero momentum
 *                                               13 was area when using the SLOW_DRIVE (small momentum)
 *
 * WEIGHT - A heavier bot will change the amount of momentum at MAX_DRIVE and SLOW_DRIVE which effects
 *          the DESIRED_TARGET_AREA_TELEOP and DESIRED_TARGET_AREA_SLOW_TELEOP
 *          A heavier bot may need higher MAX_DRIVE and SLOW_DRIVE
 * 
 * CALIBRATION STEPS
 * =================
 *  
 */



/*
*Numbers for Lime bot (need to change back if using Lime bot.)
* final double SLOW_DRIVE = 0.19; (slow speed)
* final double SLOW_VELOCITY = 400;
* final double SLOW_TARGET_APPROACH_ZONE_AREA = 3.0;
* final double DESIRED_TARGET_AREA_SLOW_TELEOP = 13; 
* ADDED OFFSET_TX to correct for bent/buckled bot
* final double SAMPLE_ZONE_AREA = 10.0;
*/

  double heading=0;
  double distance=0;

  private double tv = 0.0;
  private double tx = 0.0;
  private double ty = 0.0;
  private double ta = 0.0;
  private double lastTx = RESET_LAST_TX_VALUE;
  private double lastTa = RESET_LAST_TA_VALUE;
 
  private boolean m_LimelightHasValidTarget = false;
  private double m_LimelightDriveCommand = 0.0;
  private double m_LimelightAngleCommand = 0.0;
  private double m_LimelightTargetArea = 0.0;
  private boolean isAutonomous=false;        //Need to always hit targets during autonomous;
  private boolean isFinished=false;
  private boolean firstTime=true;

  BobDriveHelper helper;
  private Notifier visionTrackLoop=null; // Notifier (runnable) Feed MP Executer runs at least twice as fast the fastest td (time duration) that you plan to use eg 10ms


	/*
	 * Motor Controllers
	 */
	private BobTalonSRX rightTalon = Robot.drivetrain.rightLead;
	private BobTalonSRX leftTalon = Robot.drivetrain.leftLead;
	private StickyFaults faults = new StickyFaults();


  public LimeLightTrackPIDToAreaLS() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this(false);
  }  


  public LimeLightTrackPIDToAreaLS(boolean isAutonomous) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.drivetrain);
    helper = new BobDriveHelper();
    this.isAutonomous = isAutonomous;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    System.out.println("Starting LimelightPIDTrackToArea");
    m_LimelightHasValidTarget = false;
    m_LimelightDriveCommand = 0.0;
    m_LimelightAngleCommand = 0.0;
    m_LimelightTargetArea = (isAutonomous?DESIRED_TARGET_AREA_AUTON:DESIRED_TARGET_AREA_TELEOP);
    isFinished=false;
    firstTime=true;
    lastTx=RESET_LAST_TX_VALUE;
    lastTa=RESET_LAST_TA_VALUE;
    this.visionTrackLoop = new Notifier(new LimeLightVisionTrackProcessor());
    this.visionTrackLoop.startPeriodic(0.005);
    resetTalon(rightTalon, ControlMode.PercentOutput, 0);
    resetTalon(leftTalon, ControlMode.PercentOutput, 0);
    
    setUpTalon(rightTalon);
  }


  private class LimeLightVisionTrackProcessor implements java.lang.Runnable {	

    public void run() {

      limelightTrackForDriveAngleValues();
      if (m_LimelightHasValidTarget) {
        rightTalon.set(ControlMode.PercentOutput, m_LimelightDriveCommand, DemandType.AuxPID, 10 * m_LimelightAngleCommand);
        leftTalon.follow(rightTalon, FollowerType.AuxOutput1);
       
        if (firstTime) {
          rightTalon.selectProfileSlot(Drivetrain.ROTATION_PROFILE,rightTalon.getSecondaryPidIndex());
          
          firstTime=false;
        } 
        
        rightTalon.getStickyFaults(faults);
        rightTalon.configAuxPIDPolarity(false, 0);
        if (faults.UnderVoltage) {
          System.out.println("ERROR: MP Shutting Down, UNDER VOLTAGE");
          //Robot.drivetrain.isClosedLoopFailure = true;
        }
        if (faults.RemoteLossOfSignal) {
          System.out.println("ERROR: MP Shutting Down, REMOTE SIGNAL LOST");
          //Robot.drivetrain.isClosedLoopFatailure = true;
        }
        if (faults.SensorOutOfPhase) {
          System.out.println("ERROR: MP Shutting Down, SENSOR OUT OF PHASE");
          //Robot.drivetrain.isClosedLoopFatailure = true;
        }
        if (faults.ResetDuringEn) {
          System.out.println("ERROR: MP Shutting Down, RESET DURING EN");
          //Robot.drivetrain.isClosedLoopFailure = true;
        }
        SmartDashboard.putNumber("Limelight Track To Current Angle", Robot.drivetrain.getNavAngle());
        SmartDashboard.putNumber("Limelight Track To Target Angle",  m_LimelightAngleCommand);
        SmartDashboard.putNumber("Limelight Track To Drive Amount",  m_LimelightDriveCommand);
        SmartDashboard.putNumber("Limelight Track to Current Area",  ta);
        SmartDashboard.putNumber("Limelight Track to Target Area",   m_LimelightTargetArea);
        SmartDashboard.putNumber("Limelight Track to Master Talon Angle",   rightTalon.getSecondarySensorPosition());
      } else if (isAutonomous) {
        isFinished = true;
        System.out.println("LimelightTrackPIDToArea is Finished");
      } else {
        rightTalon.set(ControlMode.PercentOutput, 0, DemandType.AuxPID, 10 * m_LimelightAngleCommand);
        leftTalon.follow(rightTalon, FollowerType.AuxOutput1); 
      }
    }
	}

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  public void limelightTrackForDriveAngleValues()
  {
        lastTx = tx;
        tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

        if (tv < 1.0)
        {
          m_LimelightHasValidTarget = false;
          m_LimelightDriveCommand = 0.0;
          //m_LimelightAngleCommand = 0.0;
          lastTx = RESET_LAST_TX_VALUE;
          return;
        }

        /*
         *  Throw Away TX changes that are ridiculous
         * 
         * nb: deemed a bit too dangerous, better off without this...
         *     gets stuck in an infinite loop during Auto
         */
        /*
        if ((lastTx != RESET_LAST_TX_VALUE) && (Math.abs(lastTx-tx)>ACCEPTABLE_TX_DELTA)) {
          System.out.println("Limelight Tracking - Ditched last tx, Delta too large.");
          tx = lastTx; //re-use the last sensible tx;
        }*/

        /*
         * If we have stopped moving 
         */
        if (lastTa == ta) {
          m_LimelightHasValidTarget = false;
          m_LimelightDriveCommand = 0.0;
          //m_LimelightAngleCommand = 0.0;
          return;
        } else {
          m_LimelightHasValidTarget = true;
        }
      
        // Start with proportional steering
        double angle_cmd = Robot.drivetrain.getNavAngle() - tx;
        
        // only change angle either when it's the first time and when with in sample area (to be tested out). 
        if((ta >= SAMPLE_ZONE_AREA) || firstTime){ 
          // do sample
          m_LimelightAngleCommand = angle_cmd;
          } else {
            //don't sample
          } 
        


        // try to drive forward until the target area reaches our desired area
        double drive_cmd;

        /*
         * SLOW TRACKING
         * If the robot start quite close, then we can allow slow tracking to target
         * The DESIRED_TARGET AREA can be larger before needing to stop (less momentum if going slow).
         */
        double speed=Math.abs(Robot.drivetrain.getVelocity());
   
        if ((speed < SLOW_VELOCITY) &&
            (ta > SLOW_TARGET_APPROACH_ZONE_AREA) ) {
              System.out.printf("SLOW MODE: %s u/100ms\r",speed);
   
              if (isAutonomous) {
                  drive_cmd = (DESIRED_TARGET_AREA_SLOW - ta) * SLOW_DRIVE_K; //SLOW and AUTON go to actual target
                  m_LimelightTargetArea = DESIRED_TARGET_AREA_SLOW;
              } else {  
                  drive_cmd = (DESIRED_TARGET_AREA_SLOW_TELEOP - ta) * SLOW_DRIVE_K; //TELEOP go to 185mm short of target
                  m_LimelightTargetArea = DESIRED_TARGET_AREA_TELEOP;
              }
              if (drive_cmd > SLOW_DRIVE) {
                  drive_cmd = SLOW_DRIVE;          
              } else {
                  drive_cmd = 0;
              }
        } else {
          System.out.printf("FAST MODE: %s u/100ms\r",speed);
            if (isAutonomous) {
                drive_cmd = (DESIRED_TARGET_AREA_AUTON - ta) * DRIVE_K;
                m_LimelightTargetArea = DESIRED_TARGET_AREA_AUTON;
            } else {
                drive_cmd = (DESIRED_TARGET_AREA_TELEOP - ta) * DRIVE_K;  
                m_LimelightTargetArea = DESIRED_TARGET_AREA_TELEOP;
            }

            if (drive_cmd > MAX_DRIVE)  {
                drive_cmd = MAX_DRIVE;
            } else {
              drive_cmd = 0;
            }
        }
        m_LimelightDriveCommand = drive_cmd;
  }


	private void setUpTalon(TalonSRX talon) {
		talon.setStatusFramePeriod(StatusFrame.Status_12_Feedback1,20);
		talon.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0,20);
		talon.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1,20);
        //Put the following on drivetrain.resetPigeon
		//this.pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR,5);
		talon.configClosedLoopPeriod(0,1);  //up this timeout if remote encoder is too slow giving us updates
		talon.configClosedLoopPeriod(1,1);  //up this timeout if remote pigeon is too slow giving us updates
	}


  private void resetTalon(TalonSRX talon, ControlMode controlMode, double setCMValue) {
		talon.set(controlMode, setCMValue);
	}

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return isFinished;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    if (this.visionTrackLoop != null)
     {
      this.visionTrackLoop.stop();
      this.visionTrackLoop.close();
    }
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    if (this.visionTrackLoop != null) {
      this.visionTrackLoop.stop();
      this.visionTrackLoop.close();
    }
  }
}
