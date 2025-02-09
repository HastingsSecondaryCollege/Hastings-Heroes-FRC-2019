/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class ExtensionRailSubsystem extends Subsystem {
  public boolean IsRailExtended = false;
  public boolean IsRailOnTarget = false;
  public double maxTolerance;
  public double minTolerance;
  
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void railMoveTo(double railPosition) {
    Robot.TalonExtensionRail.set(ControlMode.Position, railPosition);
      System.out.println("Moving rail to");
      System.out.println(railPosition);
        
    }

  }
