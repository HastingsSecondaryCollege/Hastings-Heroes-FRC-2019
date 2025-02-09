/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class ResetHabEncoder extends Command {
  public ResetHabEncoder() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires (Robot.habLiftSub);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    System.out.println("Reset Hab Lift Started");
    Robot.TalonHabLift.set(ControlMode.PercentOutput, RobotMap.HAB_INIT_POWER);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.TalonHabLift.getSensorCollection().isFwdLimitSwitchClosed();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.TalonHabLift.setSelectedSensorPosition(0);
    Robot.TalonHabLift.set(ControlMode.Position, RobotMap.ENCODER_HAB_LIFT_RETRACTED);
    System.out.println("Hab Lift Fully Retracted");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
