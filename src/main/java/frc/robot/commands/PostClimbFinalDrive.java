/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.models.DriveSignal;
import frc.utils.BobDriveHelper;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class PostClimbFinalDrive extends Command {
  BobDriveHelper helper;

  public PostClimbFinalDrive() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    //MDE20190310 requires(Robot.driveTrainSub);
    requires(Robot.drivetrain);
    helper = new BobDriveHelper();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.timeyPostClimbDrive.reset();
    Robot.timeyPostClimbDrive.start();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    //MDE20190210 Robot.driveTrainSub.curvatureDrive(RobotMap.HAB_DRIVETRAIN_POST_SPEED, RobotMap.HAB_DRIVETRAIN_ANGLE);
		DriveSignal driveSignal = helper.cheesyDrive(-RobotMap.HAB_DRIVETRAIN_POST_SPEED, RobotMap.HAB_DRIVETRAIN_ANGLE, false, false);
    Robot.drivetrain.drive(ControlMode.PercentOutput, driveSignal);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.timeyPostClimbDrive.get() >= RobotMap.POST_CLIMB_FINAL_DRIVE_TIME;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    new RequireAllSubsystems().start();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
