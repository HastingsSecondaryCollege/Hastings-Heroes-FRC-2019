/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class GoRightLevel1ToRightCargoFront extends CommandGroup {

  public GoRightLevel1ToRightCargoFront() {
    System.out.println("GoRightLevel1ToRightCargoFront Started");
    Robot.drivetrain.isClosedLoopFailure = false;
    addSequential(new ResetGyro(90));
    addSequential(new HatchPanelHold());  
    FollowArc.FollowArcTrajectory[] trajectoriesToJoin = new FollowArc.FollowArcTrajectory[1];
    trajectoriesToJoin[0] = new FollowArc.FollowArcTrajectory();
    trajectoriesToJoin[0].trajectoryFilename = "/home/lvuser/motionprofiles/RightLevel1ToRightCargoFront.dat";
    trajectoriesToJoin[0].td = 60;
    trajectoriesToJoin[0].usePigeonIMU = true;
    trajectoriesToJoin[0].inReverse = false;
    addSequential(new FollowArc(trajectoriesToJoin));
    addSequential(new LimeLightTrackPIDToArea(true)); 
  }
}
