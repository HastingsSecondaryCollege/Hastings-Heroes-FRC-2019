/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class GoLeftLevel2ToLeftCargoFront extends CommandGroup {
  /**
   * Add your docs here.
   */
  public GoLeftLevel2ToLeftCargoFront() {
    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.
    addSequential(new ResetGyro(90));
    FollowArc.FollowArcTrajectory[] trajectoriesToJoin = new FollowArc.FollowArcTrajectory[1];
    trajectoriesToJoin[0] = new FollowArc.FollowArcTrajectory();
    trajectoriesToJoin[0].trajectoryFilename = "/home/lvuser/motionprofiles/LeftLevel2ToLeftCargoFront.dat";
    trajectoriesToJoin[0].td = 60;
    trajectoriesToJoin[0].usePigeonIMU = true;
    trajectoriesToJoin[0].inReverse = false;
    /*trajectoriesToJoin[1] = new FollowArc.FollowArcTrajectory();
    trajectoriesToJoin[1].trajectoryFilename = "/home/lvuser/motionprofiles/LeftReversedFromLoadingBayToLeftCargoFront.dat";
    trajectoriesToJoin[1].td = 35;
    trajectoriesToJoin[1].usePigeonIMU = true;
    trajectoriesToJoin[1].inReverse = false;*/
    addSequential(new FollowArc(trajectoriesToJoin));
    addSequential(new LimeLightTrackPIDToArea(true)); 
  }
}
