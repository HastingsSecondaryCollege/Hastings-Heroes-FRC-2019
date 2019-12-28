/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.RobotMap;
import frc.robot.Robot;

public class ClimbCancel extends Command {
  public ClimbCancel() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.elevatorSub);
    requires(Robot.habLiftSub);
    requires(Robot.habDriveSub);
    requires(Robot.drivetrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.drivetrain.drive(ControlMode.PercentOutput, RobotMap.STOP_DOUBLE, RobotMap.STOP_DOUBLE);
    Robot.TalonElevator.set(ControlMode.Position, RobotMap.ENCODER_LEVEL_THREE_CARGO);
    Robot.VictorHabDrive.set(ControlMode.PercentOutput, RobotMap.STOP_DOUBLE);
    Robot.TalonHabLift.set(ControlMode.Position, RobotMap.ENCODER_HAB_LIFT_RETRACTED);
    Robot.railSub.railMoveTo(RobotMap.RAIL_EXTENDED);
    new ResetTimers().start();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return true;
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
