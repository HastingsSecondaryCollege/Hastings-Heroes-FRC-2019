/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ExtensionRailCheckPosition extends Command {
  public ExtensionRailCheckPosition() {
    requires(Robot.railSub);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(Robot.TalonExtensionRail.getSelectedSensorPosition() >= Robot.railSub.minTolerance && Robot.TalonExtensionRail.getSelectedSensorPosition() <= Robot.railSub.maxTolerance){
      Robot.railSub.IsRailOnTarget=true;
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.railSub.IsRailOnTarget;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    if(Robot.railSub.IsRailExtended){
      Robot.railSub.IsRailExtended = false;
    }
    else{
      Robot.railSub.IsRailExtended = true;
    }
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    if(Robot.railSub.IsRailExtended){
      Robot.railSub.IsRailExtended = false;
    }
    else{
      Robot.railSub.IsRailExtended = true;
    }
  }
}
