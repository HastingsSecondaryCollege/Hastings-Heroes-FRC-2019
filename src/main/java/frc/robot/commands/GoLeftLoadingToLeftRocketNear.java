
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class GoLeftLoadingToLeftRocketNear extends CommandGroup {
  public GoLeftLoadingToLeftRocketNear() {
    addSequential(new ResetGyro(270));
    FollowArc.FollowArcTrajectory[] trajectoriesToJoin = new FollowArc.FollowArcTrajectory[2];
    trajectoriesToJoin[0] = new FollowArc.FollowArcTrajectory();
    trajectoriesToJoin[0].trajectoryFilename = "/home/lvuser/motionprofiles/LeftLoadingBayToLeftReversedFromLoadingBay.dat";
    trajectoriesToJoin[0].td = 50;
    trajectoriesToJoin[0].usePigeonIMU = true;
    trajectoriesToJoin[0].inReverse = true;
    trajectoriesToJoin[1] = new FollowArc.FollowArcTrajectory();
    trajectoriesToJoin[1].trajectoryFilename = "/home/lvuser/motionprofiles/LeftReversedFromLoadingBayToLeftRocketNear.dat";
    trajectoriesToJoin[1].td = 50;
    trajectoriesToJoin[1].usePigeonIMU = true;
    trajectoriesToJoin[1].inReverse = false;
    addSequential(new FollowArc(trajectoriesToJoin));
    addSequential(new LimeLightTrackPIDToArea(true));
    addSequential(new LimeLightTrackPIDToArea(true));   
  }
}
