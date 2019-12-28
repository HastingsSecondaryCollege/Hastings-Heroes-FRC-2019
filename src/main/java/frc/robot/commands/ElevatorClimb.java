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
import com.ctre.phoenix.motorcontrol.ControlMode;

public class ElevatorClimb extends Command {
  public int m_ClimbLevel;
  public ElevatorClimb(int ClimbLevel) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.elevatorSub);
    m_ClimbLevel = ClimbLevel;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if (m_ClimbLevel == RobotMap.CLIMB_LEVEL_THREE) {
      Robot.TalonElevator.set(ControlMode.Position, RobotMap.ENCODER_BEFORE_LEVEL_THREE_CLIMB);
    } else { 
      Robot.TalonElevator.set(ControlMode.Position, RobotMap.ENCODER_BEFORE_LEVEL_TWO_CLIMB);
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.timeyClimbey.get() >= RobotMap.HAB_CLIMB_DELAY;  
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
      Robot.TalonElevator.configPeakOutputForward(RobotMap.ELEVATOR_MAX_CLIMB_SPEED);    
      Robot.TalonElevator.set(ControlMode.Position, RobotMap.ENCODER_LEVEL_AFTER_CLIMB);
    
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
