/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;
import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class ResetExtensionRailEncoder extends Command {
  public ResetExtensionRailEncoder() { 
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis); 
    requires(Robot.railSub);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    System.out.println("Reset Extension Rail Started");
  }

  // Called repeatedly when this Command is scheduled to run       
  @Override
  protected void execute() {
    if(Robot.TalonElevator.getSelectedSensorPosition() >= RobotMap.ENCODER_LEVEL_ZERO_GROUND - RobotMap.ELEVATOR_TOLERANCE){
      
    Robot.TalonExtensionRail.set(ControlMode.PercentOutput, RobotMap.EXTENSION_RAIL_INIT_POWER);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.TalonExtensionRail.getSensorCollection().isFwdLimitSwitchClosed();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.TalonExtensionRail.setSelectedSensorPosition(0);
    if(DriverStation.getInstance().isAutonomous()){
    Robot.TalonExtensionRail.set(ControlMode.Position, RobotMap.RAIL_EXTENDED);
    Robot.railSub.IsRailExtended = true;
    }
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
