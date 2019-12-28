/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.Notifier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class KnightRider extends Command {
  private double startHeading=0;
  private double heading=0;
  private double lastHeading=0;
  private double startDistance=0;
  private double distance=0;
  private double leftDriveCommand=0;
  private double rightDriveCommand=0;
  private boolean isFinished = false; 
  private boolean isAutonomous=false;
  private double headingOut;
  public double inoutDistance;/* inoutDistance - must be at least DISTANCE_TO_MAX_VELOCITY
                                 * is based on inoutDistance = (totalDistance - DISTANCE_TO_COMPLETE_TURN_K)/2  
                                 * eg min 3.35m with 5500 at each end 2x DISTANCE_TO_MAX_VELOCITY_K + Turning Distance
                                 * SensorUnitsPerMeter=8740 so about 2.092m to turn.
                                 */
  private Notifier knightRiderLoop=null; // Notifier (runnable) Feed MP Executer runs at least twice as fast the fastest td (time duration) that you plan to use eg 10ms


  private final double ROTATION_OVERSHOOT_FACTOR_K = 0.36; //0.32;
  private final double GYROERROR_DEGREES_K = 9; //Error in degrees that gyro is out after big 180 high G move...
  private final double DISTANCE_TO_MAX_VELOCITY_K = 5500;   //629mm min encoder pulses at each end
  private final double DISTANCE_TO_COMPLETE_TURN_K = 18284; //2.092m approx encoder pulses (in a straight line)
  private final double DISTANCE_OVERSHOOT_FACTOR_K = 0.9;
  private final double DRIVE_CORRECTION_STEP_K = 0.32; 
  
  
  private int state=0;
  /*
   * Distance from tail leaving backwards to front arriving forwards min 3.35m.
   */ 
  public KnightRider(boolean isAutonomous, double headingOut, double totalDistanceInMeters) {
    requires(Robot.drivetrain);
    this.isAutonomous = isAutonomous;
    this.headingOut = -headingOut; //extra turn as we head out of the 180.
    this.inoutDistance = ((totalDistanceInMeters*RobotMap.SensorUnitsPerMeter) - DISTANCE_TO_COMPLETE_TURN_K)/2;
    if (this.inoutDistance < DISTANCE_TO_MAX_VELOCITY_K)
       this.inoutDistance = DISTANCE_TO_MAX_VELOCITY_K;
  }
  public KnightRider(boolean isAutonomous, double headingOut) {
    this(isAutonomous,headingOut,3.35);
  }
  public KnightRider() {
    //3.35M = totalDistanceinMeters = ((2*DISTANCE_TO_MAX_VELOCITY_K)+DISTANCE_TO_COMPLETE_TURN_K)/RobotMap.SensorUnitsPerMeter;
    this(false,0,3.35);
  }


  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    System.out.println("Starting Knight Rider");
    state=0;
    isFinished =false;
    startHeading = Robot.drivetrain.getNavAngle();
    //startHeading = RobotMap.ahrs.getAngle();
    heading = startHeading;
    startDistance = Robot.drivetrain.getDistance();
    System.out.printf("startDistance: %s, startHeading: %s\r",startDistance, startHeading);
    this.knightRiderLoop = new Notifier(new KnightRiderProcessor());
    this.knightRiderLoop.startPeriodic(0.005);		
  }


  private class KnightRiderProcessor implements java.lang.Runnable {	
		public void run() {
      switch (state) {
        case 0: //reversing, fast to max velocity over shortest distance
            leftDriveCommand = -1.0;  
            rightDriveCommand = -1.0;
            distance = Robot.drivetrain.getDistance();
            if (distance < (startDistance-(inoutDistance * DISTANCE_OVERSHOOT_FACTOR_K))) {
               leftDriveCommand = 1.0;  
               rightDriveCommand = -1.0;  
               //leftDriveCommand = -1.0;  
               //rightDriveCommand = 1.0;  

               state = 1;
            }
            break;
        case 1: //Sharp right turn (full lock)
             leftDriveCommand = 1.0;  
             rightDriveCommand = -1.0;  
             //leftDriveCommand = -1.0;  
             //rightDriveCommand = 1.0;  

             heading = Robot.drivetrain.getNavAngle();
             //heading = RobotMap.ahrs.getAngle();
             if (heading < (startHeading-(180 * ROTATION_OVERSHOOT_FACTOR_K))) {
                leftDriveCommand = -0.4;  
                rightDriveCommand = 0.4;  
                //leftDriveCommand = 0.5;  
                //rightDriveCommand = -0.5;  

                state = 2;   
             } 
             
            break;
        case 2: //wait for turn to complete
             lastHeading = heading;
             heading = Robot.drivetrain.getNavAngle();
             //heading = RobotMap.ahrs.getAngle();
             if (heading < (startHeading - 180) || (lastHeading < heading)) {
               startDistance = Robot.drivetrain.getDistance(); 
               leftDriveCommand = 1.0;  
               rightDriveCommand = 1.0;
               state = 3;
             }
             break;
        case 3: //forwards, fast to max velocity over shortest distance
                heading = Robot.drivetrain.getNavAngle();        
                //heading = RobotMap.ahrs.getAngle();
                if (heading < (startHeading - 180 + headingOut + GYROERROR_DEGREES_K)) {
                   leftDriveCommand =  1 - DRIVE_CORRECTION_STEP_K;
                   rightDriveCommand = 1;
                   //leftDriveCommand = 1;
                   //rightDriveCommand = 1 - DRIVE_CORRECTION_STEP_K;

                  } else if (heading > (startHeading - 180 + headingOut + GYROERROR_DEGREES_K)) {
                  leftDriveCommand = 1;
                  rightDriveCommand = 1 - DRIVE_CORRECTION_STEP_K;      
                  //leftDriveCommand = 1 - DRIVE_CORRECTION_STEP_K;
                  //rightDriveCommand = 1;      
                } else {
                  leftDriveCommand = 1;
                  rightDriveCommand = 1;
                }
    
                distance = Robot.drivetrain.getDistance();
                if (distance > (startDistance+(inoutDistance * DISTANCE_OVERSHOOT_FACTOR_K))) {
                  leftDriveCommand = 0.0;  
                  rightDriveCommand = 0.0;
                  state = 4;
                }  
             break;          
        case 4: //done
             leftDriveCommand = 0.0;  
             rightDriveCommand = 0.0;
             if (isAutonomous) {  //Autonomous needs a finish, whilePressed doesn't
                 isFinished=true;
             }    
             break;     
      }
      System.out.printf("distance: %s, heading: %s\r",distance, heading);
      Robot.drivetrain.drive(ControlMode.PercentOutput, leftDriveCommand, rightDriveCommand);  
    }
  }


  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return isFinished;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    System.out.println("Finished Knight Rider");
    if (this.knightRiderLoop != null)
    {
     this.knightRiderLoop.stop();
     this.knightRiderLoop.close();
   }

  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    System.out.println("Finished Knight Rider");
    if (this.knightRiderLoop != null)
    {
     this.knightRiderLoop.stop();
     this.knightRiderLoop.close();
   }

  }
}
