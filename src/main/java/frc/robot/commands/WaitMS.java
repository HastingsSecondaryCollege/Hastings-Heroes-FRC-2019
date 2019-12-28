/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.Notifier;

public class WaitMS extends Command {
  private double waitTime=0;
  private Notifier waitLoop=null;
  private boolean isFinished = false;
  
  public WaitMS(double waitMS) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.waitTime = waitMS/1000;
    this.isFinished = false;
  }

  private class WaitLoopProcessor implements java.lang.Runnable {	

		public void run() {
      System.out.printf("Waited %sms\r",waitTime);
      isFinished = true;
    }
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    this.waitLoop = new Notifier(new WaitLoopProcessor());
    this.waitLoop.startSingle(this.waitTime);	
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return isFinished;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    waitLoop.stop();
    waitLoop.close();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    waitLoop.stop();
    waitLoop.close();
  }
}
