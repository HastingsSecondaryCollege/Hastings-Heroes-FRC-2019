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
import frc.robot.RobotMap;


/**
 * Add your docs here.
 */
public class ElevatorLift extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    // NO DEFAULTY COMMAND REQUIRED
  }
  public void elevatorMoveTo(int elevatorPosition){
//
  
    switch(elevatorPosition){
      default:
      
      
     
        
      break;
      case 1:
        
       Robot.TalonElevator.set(ControlMode.Position, RobotMap.ENCODER_LEVEL_ZERO_GROUND);
      // System.out.println("ENCODER_LEVEL_ZERO_GROUND");
      break;
      case 2:
       
      Robot.TalonElevator.set(ControlMode.Position, RobotMap.ENCODER_LEVEL_ONE_HATCH);
      System.out.println("ENCODER_LEVEL_ONE_HATCH");
      break;
      case 3:

      Robot.TalonElevator.set(ControlMode.Position, RobotMap.ENCODER_LEVEL_ONE_CARGO_CARGOSHIP);
      System.out.println("ENCODER_LEVEL_ONE_CARGO_CARGOSHIP");
      break;
      case 4:
      Robot.TalonElevator.set(ControlMode.Position, RobotMap.ENCODER_LEVEL_ONE_CARGO_ROCKET);
       
      System.out.println("ENCODER_LEVEL_ONE_CARGO_ROCKET");
      break;
      case 5:
      Robot.TalonElevator.set(ControlMode.Position, RobotMap.ENCODER_LEVEL_TWO_HATCH);
       
      System.out.println("ENCODER_LEVEL_TWO_HATCH");
      break;
      case 6:
      Robot.TalonElevator.set(ControlMode.Position, RobotMap.ENCODER_LEVEL_TWO_CARGO);
       
      System.out.println("ENCODER_LEVEL_TWO_CARGO");
      break;
      case 7:
      Robot.TalonElevator.set(ControlMode.Position, RobotMap.ENCODER_LEVEL_THREE_HATCH);
       
     // System.out.println("ENCODER_LEVEL_THREE_HATCH");
      break;
      case 8:
    Robot.TalonElevator.set(ControlMode.Position, RobotMap.ENCODER_LEVEL_THREE_CARGO);
    System.out.println("ENCODER_LEVEL_THREE_CARGO");    
                                                                                         
      break;
      case 9:
      // Before Level 2 climb
       Robot.TalonElevator.set(ControlMode.Position, RobotMap.ENCODER_BEFORE_LEVEL_TWO_CLIMB);
       System.out.println("Level 2 cimb");
      
      break;
      case 10:
     //  Before Level 3 climb
     Robot.TalonElevator.set(ControlMode.Position, RobotMap.ENCODER_BEFORE_LEVEL_THREE_CLIMB);
       
      System.out.println("Level 3 Climb");
      break;
      case 11:
      // Drive with cargo ball
      Robot.TalonElevator.set(ControlMode.Position, RobotMap.ENCODER_DRIVE_WITH_CARGO);
    }
 }
}








































































































































































































































