/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.RobotMap;
import frc.robot.Robot;

public class DisableElevatorTopDelay extends Command {
  public DisableElevatorTopDelay() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.timeyDisableElevatorTopDelay.reset();
    Robot.timeyDisableElevatorTopDelay.start();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.timeyDisableElevatorTopDelay.get() >= RobotMap.DISABLE_ELEVATOR_TOP_TRIGGER_DELAY;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.timeyDisableElevatorTopDelay.stop();
    Robot.timeyDisableElevatorTopDelay.reset();
    if (RobotMap.xboxController.getRawAxis(2) >= RobotMap.DISABLE_ELEVATOR_TOP_TRIGGER_DISTANCE){

      new DisableElevatorTopCommand().start();
  
    }

  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
