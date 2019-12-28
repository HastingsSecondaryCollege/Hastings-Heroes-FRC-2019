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
import frc.robot.subsystems.HabDriveSubsystem;

import frc.models.DriveSignal;
import frc.utils.BobDriveHelper;
import com.ctre.phoenix.motorcontrol.ControlMode;



public class PostClimbDriveTrain extends Command {
  BobDriveHelper helper;
  public int m_ClimbLevel;

  public PostClimbDriveTrain(int  ClimbLevel) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    //MDE20190310 requires(Robot.driveTrainSub);
    requires(Robot.drivetrain);
    helper = new BobDriveHelper();
    m_ClimbLevel = ClimbLevel;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
      //MDE20190310 DriveTrainSubsystem.habAutoDrive= true;
      Robot.drivetrain.habAutoDrive= true;
      Robot.habDriveSub.TimerStarted = false;
      Robot.habDriveSub.ReadyForHabDrive = false;
    }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() { 
    if (m_ClimbLevel == RobotMap.CLIMB_LEVEL_THREE) {
     if((Robot.TalonElevator.getSelectedSensorPosition() >= (RobotMap.ENCODER_LEVEL_ZERO_GROUND - RobotMap.ELEVATOR_CLIMB_TOLERANCE)) && (Robot.TalonHabLift.getSelectedSensorPosition() <= (RobotMap.ENCODER_HAB_LIFT_LEVEL_THREE + RobotMap.HAB_LIFT_CLIMB_TOLERANCE))) {
  // if(Robot.TalonHabLift.getSelectedSensorPosition() <= (RobotMap.ENCODER_HAB_LIFT_LEVEL_THREE + RobotMap.HAB_LIFT_CLIMB_TOLERANCE)) {
        Robot.habDriveSub.ReadyForHabDrive = true; 
    }
  }
else{
    if((Robot.TalonElevator.getSelectedSensorPosition() >= (RobotMap.ENCODER_LEVEL_ZERO_GROUND - RobotMap.ELEVATOR_CLIMB_TOLERANCE)) && (Robot.TalonHabLift.getSelectedSensorPosition() <= (RobotMap.ENCODER_HAB_LIFT_LEVEL_TWO + RobotMap.HAB_LIFT_CLIMB_TOLERANCE))) {
      Robot.habDriveSub.ReadyForHabDrive = true;
}
}
    if(Robot.habDriveSub.ReadyForHabDrive == true){
    DriveSignal driveSignal = helper.cheesyDrive(-RobotMap.HAB_DRIVETRAIN_POST_SPEED, RobotMap.HAB_DRIVETRAIN_ANGLE, false, false);
    Robot.drivetrain.drive(ControlMode.PercentOutput, driveSignal);

       if (Robot.habDriveSub.TimerStarted == false){
        Robot.timeyPostClimbDrive.stop();
        Robot.timeyPostClimbDrive.reset();
        Robot.timeyPostClimbDrive.start();
        System.out.println("Timer Start");
        Robot.habDriveSub.TimerStarted = true;
     }
  }
  else {
    System.out.println("Too low");
    //MDE20190310 Robot.driveTrainSub.curvatureDrive(RobotMap.HAB_DRIVETRAIN_PRE_SPEED, RobotMap.HAB_DRIVETRAIN_ANGLE);
    DriveSignal driveSignal = helper.cheesyDrive(-RobotMap.HAB_DRIVETRAIN_PRE_SPEED, RobotMap.HAB_DRIVETRAIN_ANGLE, false, false);
    Robot.drivetrain.drive(ControlMode.PercentOutput, driveSignal);
  }
  }
  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.timeyPostClimbDrive.get() >= RobotMap.POST_CLIMB_DRIVE_TIME;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    System.out.println("Ended");
    //MDE20190310 DriveTrainSubsystem.habAutoDrive = false; 
   // Robot.drivetrain.habAutoDrive = false; // leave true, don't want to enable joystick drive
   // Robot.driveTrainSub.curvatureDrive(RobotMap.STOP_DOUBLE, RobotMap.HAB_DRIVETRAIN_ANGLE);
    new HabClimbRetract().start();
    Robot.timeyPostClimbDrive.stop();
    Robot.timeyPostClimbDrive.reset();
        
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
