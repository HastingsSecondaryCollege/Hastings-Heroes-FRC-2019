/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.command.Command;

public class CargoEject extends Command {
  
  public CargoEject() {
    requires(Robot.intakeWheelsSub);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.timeyCargoEjecty.reset();
    Robot.timeyCargoEjecty.start();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.intakeWheelsSub.CargoWheels(RobotMap.CARGO_EJECT_POWER); 
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.timeyCargoEjecty.get() >= RobotMap.CARGO_EJECT_DURATION; 
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.intakeWheelsSub.CargoWheels(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.intakeWheelsSub.CargoWheels(0);
  }
}
