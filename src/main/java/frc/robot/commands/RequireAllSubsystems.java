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

public class RequireAllSubsystems extends Command {
  public RequireAllSubsystems() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.disableElevatorTopSub);
    requires(Robot.drivetrain);
    requires(Robot.elevatorSub);
    requires(Robot.habDriveSub);
    requires(Robot.habLiftSub);
    requires(Robot.hatchPanelClampSub);
    requires(Robot.intakeWheelsSub);
    requires(Robot.railSub);

    
  }
  
  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    setInterruptible(false);
    Robot.drivetrain.drive(ControlMode.PercentOutput, RobotMap.STOP_DOUBLE, RobotMap.STOP_DOUBLE);
    Robot.TalonElevator.set(ControlMode.Position, RobotMap.ENCODER_LEVEL_ZERO_GROUND);
    Robot.VictorHabDrive.set(ControlMode.PercentOutput, RobotMap.STOP_DOUBLE);
    Robot.TalonHabLift.set(ControlMode.Position, RobotMap.ENCODER_HAB_LIFT_RETRACTED);
    Robot.SolenoidHatch.set(0); 
    Robot.intakeWheelsSub.CargoWheels(0);
    Robot.railSub.railMoveTo(RobotMap.RAIL_RETRACTED);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
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
