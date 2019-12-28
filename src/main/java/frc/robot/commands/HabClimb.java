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

public class HabClimb extends Command {
  public int m_ClimbLevel;
  public HabClimb(int ClimbLevel) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.habLiftSub);
    m_ClimbLevel = ClimbLevel;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.timeyClimbey.reset();
    Robot.timeyClimbey.start();

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
   return Robot.timeyClimbey.get()>=RobotMap.HAB_CLIMB_DELAY;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    if (m_ClimbLevel == RobotMap.CLIMB_LEVEL_THREE) {
      Robot.TalonHabLift.set(ControlMode.Position, RobotMap.ENCODER_HAB_LIFT_LEVEL_THREE);
    } else {
      Robot.TalonHabLift.set(ControlMode.Position, RobotMap.ENCODER_HAB_LIFT_LEVEL_TWO);
    } 
    
    Robot.timeyClimbey.stop();
    Robot.habLiftSub.climbStarted = true;
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.timeyClimbey.stop();
  }
}
