/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.Robot;
import frc.robot.RobotMap;

import frc.models.DriveSignal;
import frc.robot.subsystems.Drivetrain;
import frc.utils.BobDriveHelper;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.Notifier;


//temp
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

public class LimeLightTrackToArea extends Command {

  private boolean m_LimelightHasValidTarget = false;
  private boolean m_ShiftingLeftRotateLeft = false;
  private boolean m_ShiftingLeftStraightAhead = false;
  private boolean m_ShiftingLeftRotateRight = false;
  private boolean m_ShiftingRightRotateRight = false;
  private boolean m_ShiftingRightStraightAhead = false;
  private boolean m_ShiftingRightRotateLeft = false;
  private boolean m_ShiftCompleted = false;
  private boolean m_ShiftEarlyRollReadingCompleted = false;
  private boolean m_IsQuickTurn = false;
  private double m_LimelightDriveCommand = 0.0;
  private double m_LimelightSteerCommand = 0.0;
  private double m_ShiftHeading = 0.0;
  private double m_ShiftStraightAhead = 0.0; //calculated using current distance and m_ShiftDistance after first rotation 
  private double m_ShiftDistance = 0.0;      //calculated at time of Tracking commencement
  private boolean isAutonomous=false;        //Need to always hit targets during autonomous;
  private boolean isFinished=false;
  private double m_last_ta=0;
  BobDriveHelper helper;
  private Notifier visionTrackLoop=null; // Notifier (runnable) Feed MP Executer runs at least twice as fast the fastest td (time duration) that you plan to use eg 10ms

  double tv=0.0;
  double tx=0.0;
  double ty=0.0;
  double ta=0.0;


  public LimeLightTrackToArea() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this(false);
  }  


  public LimeLightTrackToArea(boolean isAutonomous) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.drivetrain);
    System.out.println("LimeLightTrackToArea Constructor");
    helper = new BobDriveHelper();
    this.isAutonomous = isAutonomous;

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    System.out.println("Starting LimilightTrackToArea");
    m_LimelightHasValidTarget = false;
    m_ShiftingLeftRotateLeft = false;
    m_ShiftingLeftStraightAhead = false;
    m_ShiftingLeftRotateRight = false;
    m_ShiftingRightRotateRight = false;
    m_ShiftingRightStraightAhead = false;
    m_ShiftingRightRotateLeft = false;
    m_IsQuickTurn = false;
    m_LimelightDriveCommand = 0.0;
    m_LimelightSteerCommand = 0.0;
    m_ShiftHeading = 0.0;
    m_ShiftStraightAhead = 0.0;
    m_ShiftDistance = 0.0;
    m_ShiftCompleted = false;
    m_ShiftEarlyRollReadingCompleted = false;
    m_last_ta = 0;
    isFinished=false;
    /*TalonSRXConfiguration allConfigs = new TalonSRXConfiguration();
    Robot.drivetrain.rightLead.getAllConfigs(allConfigs);
    System.out.println(allConfigs.toString("Master Talon Config: "));*/
    this.visionTrackLoop = new Notifier(new LimeLightVisionTrackProcessor());
    this.visionTrackLoop.startPeriodic(0.005);		
  }


  private class LimeLightVisionTrackProcessor implements java.lang.Runnable {	

		public void run() {
      
      limelightTrackForDriveSteerValues();
      
      if (m_LimelightHasValidTarget || m_ShiftingRightStraightAhead || m_ShiftingLeftStraightAhead){
        DriveSignal driveSignal = helper.cheesyDrive(m_LimelightDriveCommand, m_LimelightSteerCommand, false, false);
        Robot.drivetrain.drive(ControlMode.PercentOutput, driveSignal);
      } else if (m_ShiftingLeftRotateLeft || m_ShiftingLeftRotateRight || m_ShiftingRightRotateRight || m_ShiftingRightRotateLeft) {
            Robot.drivetrain.drive(ControlMode.PercentOutput, m_LimelightSteerCommand, -1*m_LimelightSteerCommand);
      } else if (isAutonomous) {
        isFinished = true;
        System.out.println("LimelightTrackToArea is Finished");
      }     
		}
	}

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
/*    limelightTrackForDriveSteerValues();

    if (m_LimelightHasValidTarget || m_ShiftingRightStraightAhead || m_ShiftingLeftStraightAhead){
      DriveSignal driveSignal = helper.cheesyDrive(m_LimelightDriveCommand, m_LimelightSteerCommand, false, false);
      Robot.drivetrain.drive(ControlMode.PercentOutput, driveSignal);
    } else if (m_ShiftingLeftRotateLeft || m_ShiftingLeftRotateRight || m_ShiftingRightRotateRight || m_ShiftingRightRotateLeft) {
          Robot.drivetrain.drive(ControlMode.PercentOutput, m_LimelightSteerCommand, -1*m_LimelightSteerCommand);
    } 
    }*/
  }

  /**
   * This function implements a simple method of generating driving and steering commands
   * based on the tracking data from a limelight camera.
   */
  public void limelightTrackForDriveSteerValues()
  {
        // These numbers must be tuned for your Robot!  Be careful!
        final double STEER_K = 0.010;//0.015 0.025;                    // how hard to turn toward the target
        final double DRIVE_K = 0.66;//0.26;                    // how hard to drive fwd toward the target
        final double SLOW_DRIVE_K = 0.26;               // used if starting from close at slow speed
        final double SLOW_VELOCITY = 10;
        final double DESIRED_TARGET_AREA_SLOW   = 20.0; //25.0; //29.0; //% Area* of the target when the robot reaches the wall
        final double DESIRED_TARGET_AREA_AUTON  = 6.9; //6.8; //10.60; //29.0; //% Area* of the target when the robot reaches the wall
        final double DESIRED_TARGET_AREA_TELEOP = 5.11;//7.5; //29.0; //% Area* of the target when the robot reaches the wall
                                                        //*Need to account for Speed and Overshoot ie if going fast, cutoff at 7 to stop at 29 
        final double SLOW_DRIVE = 0.19;   
        final double MAX_DRIVE = 0.4;                   // Simple speed limit so we don't drive too fast
        final double SLOWDOWN_FOR_3D_READING_DRIVE = 0.23; //.0.15;
        /*

         * ROLL - Basically 3D only works in a stable manner when you are 1.0 meters from target.
         *        We can, however get a reasonably helpful ROLL indication (and length camtranZ), a lot earlier.
         *        We can take an early camtranRoll reading, say at 2m, and decide if we are going to have to do a shift.
         *        If so, we can slow down mid tracking, get to MAX_ROLL_FOR_FINAL_TRACKING and take an accurate 3D reading
         *        Then do the shift from 1meter away.
         */
        final double MAX_ROLL_FOR_FINAL_TRACKING = 3.0;       //I accept this camtranRoll value (or under) to go ahead and do basic tracking
        final double MAX_TA_TOO_LATE_TO_START_SHIFT = 5.33; //ta=5.33 0.92m //need some room for the manouver
        final double MAX_TA_ACCURATE_3D_READING = 4.5; //ta=4.5 at 1.0m  //% Area of the target when robot is 1.0m away (3D good numbers range)
        final double MAX_TA_EARLY_ROLL_READING = 1.47;  //ta=1.47 at 1.9m //Check camtranRoll earlyvalue (or under) to go ahead and do basic tracking
        final double MAX_TX_ANGLE_BEFORE_READING_ROLL = 6.0; //3.0;  //tx needs to be close, before looking at camtranRoll 
        //final double MIN_DISTANCE_FOR_TRACKING = 0.5;
        //final double MIN_DISTANCE_FOR_SHIFT_TRACKING = 1.2;
        final double SHIFT_STEER_PER_20MS_K = 0.25; 
        final double SHIFT_DRIVE_PER_20MS_K = 0.25;
        final double ROTATE_OVERSHOOT_FACTOR_K = 0.82; //Change this is you change SHIFT STEER rate
        final double DISTANCE_OVERSHOOT_FACTOR_K = 0.54; //Change this is you change SHIFT DRIVE rate
         
        
        double heading=0;
        double distance=0;
        
        m_last_ta = ta;
        tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

        Number[] camtran;
        Number[] defaultNumbers = {0,0,0,0,0,0};
        camtran = NetworkTableInstance.getDefault().getTable("limelight").getEntry("camtran").getNumberArray(defaultNumbers);
		    //double camtranZaw = (double)camtran[4];
        //double camtranX = (double)camtran[0];
        double camtranX = (double)camtran[0];
				double camtranZ = (double)camtran[2];
        double camtranRoll = (double)camtran[5];

        /*
        if (m_ShiftingRightRotateRight) {
          heading = Robot.drivetrain.getNavAngle();
          if (heading > m_ShiftHeading) {
            m_LimelightSteerCommand = SHIFT_STEER_PER_20MS_K;    
            m_LimelightDriveCommand = SHIFT_DRIVE_PER_20MS_K;    
            System.out.printf("Right Right Heading: %s    current heading: %s\r",m_ShiftHeading, heading);            
            return;
          } else {
            m_ShiftingRightRotateRight = false;
            m_ShiftingRightStraightAhead = true;
            m_ShiftingRightRotateLeft = false;
            distance = Robot.drivetrain.getDistance();
            m_ShiftStraightAhead = distance + m_ShiftDistance;
          }
        }

        if (m_ShiftingRightStraightAhead) {
          distance = Robot.drivetrain.getDistance();
          if (distance < m_ShiftStraightAhead) {
            m_LimelightSteerCommand = 0;
            m_LimelightDriveCommand = SHIFT_DRIVE_PER_20MS_K;    
            System.out.printf("Right Straight Ahead: %s    current position: %s\r",m_ShiftStraightAhead, distance);                        
            return;            
          } else {
            m_ShiftingRightRotateRight = false;
            m_ShiftingRightStraightAhead = false;
            m_ShiftingRightRotateLeft = true;
            heading = Robot.drivetrain.getNavAngle();
            m_ShiftHeading = heading + (ROTATE_OVERSHOOT_FACTOR_K * 90);
          }
        }

        if (m_ShiftingRightRotateLeft) {
          heading = Robot.drivetrain.getNavAngle();
          if (heading < m_ShiftHeading) {
            m_LimelightSteerCommand = -1 * SHIFT_STEER_PER_20MS_K;    
            m_LimelightDriveCommand = SHIFT_DRIVE_PER_20MS_K;    
            System.out.printf("Right Left Heading: %s    current heading: %s\r",m_ShiftHeading, heading);                        
            return;            
          } else {
            m_ShiftingRightRotateRight = false;
            m_ShiftingRightStraightAhead = false;
            m_ShiftingRightRotateLeft = false;
            m_ShiftCompleted = true;
          }
        }

        if (m_ShiftingLeftRotateLeft) {
          heading = Robot.drivetrain.getNavAngle();
          if (heading < m_ShiftHeading) {
            m_LimelightSteerCommand = -1 * SHIFT_STEER_PER_20MS_K;    
            m_LimelightDriveCommand = SHIFT_DRIVE_PER_20MS_K;
            System.out.printf("Left Left Heading: %s    current heading: %s\r",m_ShiftHeading, heading); 
            return;
          } else {
            m_ShiftingLeftRotateLeft = false;
            m_ShiftingLeftStraightAhead = true;
            m_ShiftingLeftRotateRight = false;   
            distance = Robot.drivetrain.getDistance();
            m_ShiftStraightAhead = distance + m_ShiftDistance;
          }
        }

        if (m_ShiftingLeftStraightAhead) {
          distance = Robot.drivetrain.getDistance();
          if (distance < m_ShiftStraightAhead) {
            m_LimelightSteerCommand = 0;
            m_LimelightDriveCommand = SHIFT_DRIVE_PER_20MS_K;    
            System.out.printf("Left Straight Ahead: %s    current position: %s\r",m_ShiftStraightAhead, distance);                        
            return;            
          } else {
            m_ShiftingLeftRotateLeft = false;
            m_ShiftingLeftStraightAhead = false;
            m_ShiftingLeftRotateRight = true;
            heading = Robot.drivetrain.getNavAngle();
            m_ShiftHeading = heading + (ROTATE_OVERSHOOT_FACTOR_K * -90);
          }
        }

        if (m_ShiftingLeftRotateRight) {
          heading = Robot.drivetrain.getNavAngle();
          if (heading > m_ShiftHeading) {
            m_LimelightSteerCommand = SHIFT_STEER_PER_20MS_K;    
            m_LimelightDriveCommand = SHIFT_DRIVE_PER_20MS_K;    
            System.out.printf("Left Right Heading: %s    current heading: %s\r",m_ShiftHeading, heading);  
            return;
          } else {
            m_ShiftingLeftRotateLeft = false;
            m_ShiftingLeftStraightAhead = false;
            m_ShiftingLeftRotateRight = false;
            m_ShiftCompleted = true;
          }
        }
        */

        if (tv < 1.0)
        {
          m_LimelightHasValidTarget = false;
          m_LimelightDriveCommand = 0.0;
          m_LimelightSteerCommand = 0.0;
          return;
        }

        m_LimelightHasValidTarget = true;
      
        //System.out.printf("camtranRoll:%s\r",camtranRoll);
        
       /*
        * Let's see if we need to do a Shift (hard right, forward, hard left, track to target)
        */
   /****   SHIFT - not good enough for production, get back to this another time ....
    * 
        if (ta > MAX_TA_ACCURATE_3D_READING && 
           !m_ShiftCompleted && 
           (Math.abs(tx) < MAX_TX_ANGLE_BEFORE_READING_ROLL) && 
           (Math.abs(camtranRoll) > MAX_ROLL_FOR_FINAL_TRACKING)
           && ta < MAX_TA_TOO_LATE_TO_START_SHIFT) {
           System.out.printf("camtranRoll:%s\r",camtranRoll);
           m_LimelightHasValidTarget = false;
           m_IsQuickTurn = true;
           if (camtranRoll < 0) {    
               m_ShiftingRightRotateRight = true;
               heading = Robot.drivetrain.getNavAngle();
               m_ShiftHeading = heading - ROTATE_OVERSHOOT_FACTOR_K*(tx + Math.toDegrees(Math.atan(camtranZ/camtranX)));
               m_ShiftDistance = DISTANCE_OVERSHOOT_FACTOR_K*Math.abs(camtranX) * RobotMap.inches2MetersFactor * RobotMap.SensorUnitsPerMeter;
               m_LimelightSteerCommand = SHIFT_STEER_PER_20MS_K;    
               m_LimelightDriveCommand = SHIFT_DRIVE_PER_20MS_K; 
               System.out.printf("m_ShiftHeading[%s] = heading[%s] - ROTATE_OVERSHOOT_FACTOR_K[%s]*(tx[%s] + Math.toDegrees[%s](Math.atan[%s](camtranZ([%s]/camtranX[%s])))\r",m_ShiftHeading,heading, ROTATE_OVERSHOOT_FACTOR_K,tx,Math.toDegrees(Math.atan(camtranZ/camtranX)),Math.atan(camtranZ/camtranX),camtranZ,camtranX);
               System.out.printf("m_ShiftDistance[%s] = DISTANCE_OVERSHOOT_FACTOR_K[%s]*[%s]Math.abs[%s]([%s]camtranX) * RobotMap.inches2MetersFactor[%s] * RobotMap.SensorUnitsPerMeter[%s]\r",m_ShiftDistance,DISTANCE_OVERSHOOT_FACTOR_K,Math.abs(camtranX) * RobotMap.inches2MetersFactor, Math.abs(camtranX),camtranX,RobotMap.inches2MetersFactor,RobotMap.SensorUnitsPerMeter);
               System.out.printf("Face East Heading: %s,   current heading:%s followed by distance: %s then head North\r",m_ShiftHeading, heading, m_ShiftDistance);
               return;
           } else {
               m_ShiftingLeftRotateLeft = true;
               heading = Robot.drivetrain.getNavAngle();
               m_ShiftHeading = heading - ROTATE_OVERSHOOT_FACTOR_K*(tx + Math.toDegrees(Math.atan(camtranZ/camtranX)));
               m_ShiftDistance = DISTANCE_OVERSHOOT_FACTOR_K*Math.abs(camtranX) * RobotMap.inches2MetersFactor * RobotMap.SensorUnitsPerMeter;
               m_LimelightSteerCommand = -1 * SHIFT_STEER_PER_20MS_K;    
               m_LimelightDriveCommand = SHIFT_DRIVE_PER_20MS_K;
               System.out.printf("m_ShiftHeading[%s] = heading[%s] - ROTATE_OVERSHOOT_FACTOR_K[%s]*(tx[%s] + Math.toDegrees[%s](Math.atan[%s](camtranZ([%s]/camtranX[%s])))\r",m_ShiftHeading,heading, ROTATE_OVERSHOOT_FACTOR_K,tx,Math.toDegrees(Math.atan(camtranZ/camtranX)),Math.atan(camtranZ/camtranX),camtranZ,camtranX);
               System.out.printf("m_ShiftDistance[%s] = DISTANCE_OVERSHOOT_FACTOR_K[%s]*[%s]Math.abs[%s]([%s]camtranX) * RobotMap.inches2MetersFactor[%s] * RobotMap.SensorUnitsPerMeter[%s]\r",m_ShiftDistance,DISTANCE_OVERSHOOT_FACTOR_K,Math.abs(camtranX) * RobotMap.inches2MetersFactor,Math.abs(camtranX),camtranX,RobotMap.inches2MetersFactor,RobotMap.SensorUnitsPerMeter);
               System.out.printf("Face West Heading: %s,   current Heading: %s followed by distance: %s then head North\r",m_ShiftHeading, heading, m_ShiftDistance);

               return;
           }
       } else {
          if (!(ta > MAX_TA_ACCURATE_3D_READING)) {
            System.out.printf("NOT: (ta[%s] > MAX_TA_ACCURATE_3D_READING[%s])\r",ta,MAX_TA_ACCURATE_3D_READING);
          } else if (m_ShiftCompleted) {
              System.out.printf("NOT: (!m_ShiftCompleted[%s])\r",m_ShiftCompleted);
          } else if (!(Math.abs(tx) < MAX_TX_ANGLE_BEFORE_READING_ROLL)){
              System.out.printf("NOT: (Math.abs(tx[%s]) < MAX_TX_ANGLE_BEFORE_READING_ROLL[%s])\r",tx,MAX_TX_ANGLE_BEFORE_READING_ROLL);
          } else if (!(Math.abs(camtranRoll) > MAX_ROLL_FOR_FINAL_TRACKING)) {
            System.out.printf("NOT: (Math.abs(camtranRoll[%s]) > MAX_ROLL_FOR_FINAL_TRACKING[%s])\r",camtranRoll,MAX_ROLL_FOR_FINAL_TRACKING);
          } else if (!(ta < MAX_TA_TOO_LATE_TO_START_SHIFT)) {
            System.out.printf("NOT: (ta[%s] < MAX_TA_TOO_LATE_TO_START_SHIFT[%s])\r",ta,MAX_TA_TOO_LATE_TO_START_SHIFT);
          }
       }
*/
        // Start with proportional steering
        double steer_cmd = tx * STEER_K;
        m_LimelightSteerCommand = steer_cmd;

        // try to drive forward until the target area reaches our desired area
        double drive_cmd;
        boolean slowdriveFlag=false;

        /*
         * If the robot start quite close, then we canm allow slow tracking to target
         * DESIRED_TARGET AREA can be larger before needing to stop (less momentum if going slow).
         */
        double speed=Math.abs(Robot.drivetrain.getVelocity());
        //System.out.printf("VELOCITY: %s u/100ms ta: %s distance: %sm\r",speed,ta, (Math.abs(camtranZ) * RobotMap.inches2MetersFactor));

        double slowStopDistance=0;
        if (isAutonomous)
            slowStopDistance = 0;
        else  
            slowStopDistance = 0.53;

        if ((speed < SLOW_VELOCITY) &&
            (ta > MAX_TA_TOO_LATE_TO_START_SHIFT) && 
            (Math.abs(camtranZ) * RobotMap.inches2MetersFactor) > slowStopDistance) {
              System.out.printf("SLOW MODE: %s u/100ms\r",speed);
              drive_cmd = (DESIRED_TARGET_AREA_SLOW - ta) * SLOW_DRIVE_K; 
              slowdriveFlag = true;
              if (drive_cmd > SLOW_DRIVE)  
              {
                drive_cmd = SLOW_DRIVE;          
              } else {
                drive_cmd = 0;
              }
        } else {
            if (isAutonomous) {
                drive_cmd = (DESIRED_TARGET_AREA_AUTON - ta) * DRIVE_K;
            } else {
                drive_cmd = (DESIRED_TARGET_AREA_TELEOP - ta) * DRIVE_K;  
            }
        }

        // don't let the robot drive too fast into the goal
        if (!slowdriveFlag) {
            if (drive_cmd > MAX_DRIVE)  {
              /* Slow down if ROLL indicated we are coming in at an angle that will require a shift. */ 
              /*if (!m_ShiftEarlyRollReadingCompleted && (ta > MAX_TA_EARLY_ROLL_READING) && (Math.abs(camtranRoll) > MAX_ROLL_FOR_FINAL_TRACKING)) {
                drive_cmd = SLOWDOWN_FOR_3D_READING_DRIVE;
              } else {*/
                drive_cmd = MAX_DRIVE;
              //}
            } else {
              drive_cmd = 0;
            }
        }
 //       System.out.printf("Limelight Tracking... ACTIVE: %s Speed: %s Steer: %s Target Area: %s tx: %s camtranZ: %s camtranX: %s camtranZaw: %s\r",m_LimelightHasValidTarget,drive_cmd,m_LimelightSteerCommand,ta,tx,camtranZ,camtranX, camtranZaw);
        m_LimelightDriveCommand = drive_cmd;

        /* If we have stopped still in slowDrive */
        if (slowdriveFlag && (ta == m_last_ta)) {
          m_LimelightHasValidTarget = false;
        }
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
