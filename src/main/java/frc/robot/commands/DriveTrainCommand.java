package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.DriveTrainSubsystem;

import frc.models.DriveSignal;
import frc.utils.BobDriveHelper;
import com.ctre.phoenix.motorcontrol.ControlMode;


public class DriveTrainCommand extends Command {
  BobDriveHelper helper;

  public DriveTrainCommand() {
    requires(Robot.driveTrainSub);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);

  }

  // Called just before this Command runs the first time

  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run

  protected void execute() {
    if (Robot.TalonElevator.getSelectedSensorPosition() <= RobotMap.ENCODER_HIGH_DRIVE_SLOW){
      Robot.drivetrain.isElevatorHigh = true;
    } else {
         DriveTrainSubsystem.isElevatorHigh = false;
    }
     
    if (RobotMap.driveStick.getRawAxis(3)<0 && DriveTrainSubsystem.habAutoDrive == false) {
    /*
		 * Quickturn is TRUE - if your drive value is less than the Threshold 
		 * eg quickTurnThreshold = 0.5
		 * moveValue
		 * =========
		 *  0.4:      0.4  < 0.5 &&  0.4 > -0.5 ==> YES
		 * -0.4:     -0.4  < 0.5 && -0.4 > -0.5 ==> YES
		 *  0.6:      0.6  < 0.5 &&  0.6 > -0.5 ==> NO
		 * -0.6:     -0.6  < 0.5 && -0.6 > -0.5 ==> NO
		 */
		
      if (DriveTrainSubsystem.isElevatorHigh){
        Robot.driveTrainSub.curvatureDrive(RobotMap.driveStick.getY()*RobotMap.DRIVE_SLOW_SPEED_Y, RobotMap.driveStick.getZ()*RobotMap.DRIVE_SLOW_SPEED_Z);  
        /*double moveValue = RobotMap.driveStick.getY()*RobotMap.DRIVE_SLOW_SPEED_Y;
        double rotateValue = RobotMap.driveStick.getZ()*RobotMap.DRIVE_SLOW_SPEED_Z;
        boolean quickTurn = (moveValue < RobotMap.QUICKTURN_THRESHOLD && moveValue > -RobotMap.QUICKTURN_THRESHOLD);
    
        DriveSignal driveSignal = helper.cheesyDrive(-moveValue, rotateValue, quickTurn, false);
        Robot.drivetrain.drive(ControlMode.PercentOutput, driveSignal); */
      } else {
        Robot.driveTrainSub.curvatureDrive(RobotMap.driveStick.getY()*RobotMap.DRIVE_MAX_SPEED_Y, RobotMap.driveStick.getZ()*RobotMap.DRIVE_MAX_SPEED_Z);   
        /* DriveSignal driveSignal = helper.cheesyDrive(-RobotMap.driveStick.getY()*RobotMap.DRIVE_MAX_SPEED_Y, RobotMap.driveStick.getZ()*RobotMap.DRIVE_MAX_SPEED_Z, false, false);
        Robot.drivetrain.drive(ControlMode.PercentOutput, driveSignal); */
      }
    }
  }
  // make this return true when this Command no longer needs to run execute()

  protected boolean isFinished() {

    if (RobotMap.driveStick.getRawAxis(3)>0) {
      return true;
    }
      else{return false;}
    }
  // Called once after isFinished returns true
  protected void end() {
    
    Robot.driveTrainSub.curvatureDrive(0,0);
    //Robot.drivetrain.drive(ControlMode.PercentOutput, 0,0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run

  protected void interrupted() {
    Robot.driveTrainSub.curvatureDrive(0,0);
    // Robot.drivetrain.drive(ControlMode.PercentOutput, 0,0);
  }
}