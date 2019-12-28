/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.commands.DriveTrainCommand;                             
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;

public class DriveTrainSubsystem extends Subsystem {

  public static boolean habAutoDrive  = false;
  public static boolean isElevatorHigh = false; 
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public DriveTrainSubsystem(){ 
    }
    
  public void curvatureDrive(double y, double z){
    
    //MDE20190310   Robot.drive.curvatureDrive(y, z, true);
      } 

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new DriveTrainCommand());
  }
}