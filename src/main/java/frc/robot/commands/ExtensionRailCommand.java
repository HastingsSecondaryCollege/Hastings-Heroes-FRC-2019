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


public class ExtensionRailCommand extends Command {
  public ExtensionRailCommand() {
    requires(Robot.railSub);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  
  

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    System.out.println("ExtensionRailCommand initialised");
    if(Robot.TalonElevator.getSelectedSensorPosition() >= RobotMap.ENCODER_LEVEL_ZERO_GROUND - RobotMap.ELEVATOR_TOLERANCE){
      
      Robot.railSub.IsRailOnTarget = false;
      System.out.println("Elevator is down");
    if(Robot.railSub.IsRailExtended){
      System.out.println("Rail is extended");
      Robot.railSub.railMoveTo(RobotMap.RAIL_RETRACTED);
      }
      else{
        System.out.println("Rail is retracted");
      Robot.railSub.railMoveTo(RobotMap.RAIL_EXTENDED);
    }
    Robot.railSub.maxTolerance = Robot.TalonExtensionRail.getClosedLoopTarget()+(RobotMap.RAIL_TOLERANCE*-1);
  }  
}

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
   
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.railSub.maxTolerance == Robot.TalonExtensionRail.getClosedLoopTarget()+(RobotMap.RAIL_TOLERANCE*-1);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    if(Robot.TalonElevator.getSelectedSensorPosition() >= RobotMap.ENCODER_LEVEL_ZERO_GROUND - RobotMap.ELEVATOR_TOLERANCE){
    new ExtensionRailCheckPosition().start();
    }
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
