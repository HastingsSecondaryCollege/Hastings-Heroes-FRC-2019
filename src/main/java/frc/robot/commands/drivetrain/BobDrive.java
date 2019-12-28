package frc.robot.commands.drivetrain;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.models.DriveSignal;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.utils.BobDriveHelper;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class BobDrive extends Command {
	
	BobDriveHelper helper;
	private double quickTurnThreshold = RobotMap.QUICKTURN_THRESHOLD;

    long timeMSBefore=0;
    long timeMSAfter=0;


	public BobDrive() {
		requires(Robot.drivetrain);
		helper = new BobDriveHelper();
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
System.out.println("Welcome to BobDrive");
		/*
		 *  We have a trigger on "Robot.drivetrain.isClosedLoopFailure"
		 *  that will spring us out of a CommandGroup (set within FollowArc command upon error)
		 *  back to BobDrive in the event of a fatal StickyFault that we don't want to
		 *  risk continuing on.
		 * 
		 *  Once we are here, this can be reset.
		 */  
        Robot.drivetrain.isClosedLoopFailure =false; 
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		if (Robot.TalonElevator.getSelectedSensorPosition() <= RobotMap.ENCODER_HIGH_DRIVE_SLOW){
			Robot.drivetrain.isElevatorHigh = true;
		  } else {
			   Robot.drivetrain.isElevatorHigh = false;
		  }
		
		timeMSBefore = System.currentTimeMillis();
		double moveValue = Robot.m_oi.driveStick.getY();
		double rotateValue = Robot.m_oi.driveStick.getZ();

		if (Robot.drivetrain.isElevatorHigh) {
			moveValue *= RobotMap.DRIVE_SLOW_SPEED_Y;
			rotateValue *= RobotMap.DRIVE_SLOW_SPEED_Z;
		} else {
			moveValue *= RobotMap.DRIVE_MAX_SPEED_Y;
			rotateValue *= RobotMap.DRIVE_MAX_SPEED_Z;
		}
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
		boolean quickTurn = (moveValue < quickTurnThreshold && moveValue > -quickTurnThreshold);
		DriveSignal driveSignal = helper.cheesyDrive(moveValue, rotateValue, quickTurn, false);
		Robot.drivetrain.drive(ControlMode.PercentOutput, driveSignal);
		timeMSAfter = System.currentTimeMillis();
		SmartDashboard.putNumber("Time Update Drive (ms): ", timeMSAfter-timeMSBefore);		
		
//System.out.printf("Pigeon Angle: %s\r", drivetrain.getAngle());
//System.out.flush();	
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return Robot.drivetrain.habAutoDrive;
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
