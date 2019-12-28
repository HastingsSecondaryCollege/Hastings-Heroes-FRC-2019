/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class PostClimbHabDrive extends Command {
  public int m_ClimbLevel;
  
  public PostClimbHabDrive(int ClimbLevel) {
    m_ClimbLevel = ClimbLevel;
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.VictorHabDrive.set(ControlMode.PercentOutput, RobotMap.HAB_DRIVE_PRE_SPEED);
  
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(Robot.habDriveSub.ReadyForHabDrive == true) {
      Robot.VictorHabDrive.set(RobotMap.HAB_DRIVE_POST_SPEED);
    }
  }
  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.timeyPostClimbDrive.get() >= RobotMap.POST_CLIMB_DRIVE_TIME;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.VictorHabDrive.set(RobotMap.STOP_DOUBLE);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
    protected void interrupted() {
    }
  }
