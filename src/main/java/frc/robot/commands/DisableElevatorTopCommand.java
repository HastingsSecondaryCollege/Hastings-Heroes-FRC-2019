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

public class DisableElevatorTopCommand extends Command {
  public DisableElevatorTopCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.disableElevatorTopSub);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.timeyDisableElevatorTop.reset();
    Robot.timeyDisableElevatorTop.start();
    Robot.VictorDisableElevatorTop.set(RobotMap.DISABLE_ELEVATOR_TOP_POWER);
    if (Robot.TalonElevator.getSelectedSensorPosition() >= RobotMap.ENCODER_LEVEL_ZERO_GROUND - RobotMap.ELEVATOR_TOLERANCE) {
      Robot.railSub.railMoveTo(RobotMap.RAIL_EXTENDED);
    } else {
      
    }
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.timeyDisableElevatorTop.get() >= RobotMap.DISABLE_ELEVATOR_TOP_DURATION;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.VictorDisableElevatorTop.set(RobotMap.STOP_DOUBLE);
    Robot.TalonElevator.set(ControlMode.Position, RobotMap.ENCODER_LEVEL_THREE_CARGO  );
   
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
