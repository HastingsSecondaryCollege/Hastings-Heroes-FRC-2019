/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class TestCommand extends Command {
  public TestCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
      requires(Robot.habLiftSub);
    // requires(Robot.railSub);
    // requires(Robot.elevatorSub);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
   //  Robot.TalonExtensionRail.set(ControlMode.PercentOutput, RobotMap.testStick.getY()*0.5);
    // Robot.TalonElevator.set(ControlMode.PercentOutput, RobotMap.testStick.getY()*0.3);
     Robot.TalonHabLift.set(ControlMode.PercentOutput, RobotMap.testStick.getY()*0.3);

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
