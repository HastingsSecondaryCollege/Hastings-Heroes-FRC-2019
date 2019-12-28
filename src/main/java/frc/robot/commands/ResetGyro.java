package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ResetGyro extends Command {
    private static boolean finished;
    private double setAngle=0;
    
    public ResetGyro() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.drivetrain);
        this.setAngle = 90;
    }

public ResetGyro(double setAngle) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.drivetrain);
        this.setAngle=setAngle;
        System.out.println("Gyro Reset to 90 Constructor");
    }
    
    // Called just before this Command runs the first time
    protected void initialize() {
        System.out.println("Gyro Reset Initialized");  
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
      System.out.println("Gyro Reset");
      RobotMap.ahrs.reset();
     
      finished = true;
      Robot.drivetrain.resetPigeon();
      Robot.drivetrain.setNavAngle(this.setAngle);//, 0);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return finished;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
