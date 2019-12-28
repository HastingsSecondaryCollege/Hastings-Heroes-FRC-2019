/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;

public class AutoSequenceSelector extends CommandGroup {
   /*
   * Add your docs here.
   */
  String m_startingPosition;
    String m_firstDestination;
    String m_secondDestination;
  
  public AutoSequenceSelector() {  
  m_startingPosition = Robot.startingPosition.getSelected();
  m_firstDestination = Robot.firstDestination.getSelected(); 
  m_secondDestination = Robot.secondDestination.getSelected();
  
  switch (m_startingPosition){
    case "Do Nothing":
      break;
    case "Right Level 2":
      switch (m_firstDestination) { 
        case Robot.first_cargo_front:
          //mp RightLevel2ToCargoFront
           
          //mp RightCargoFrontReverse
           
          //mp RightCargoFrontToLoading
          break;
        case Robot.first_cargo_near:
          //mp 
           
          //mp 
           
          //mp 
          break;
        case Robot.first_cargo_mid:
          //mp 
           
          //mp 
           
          //mp 
          break;
        case Robot.first_cargo_far:
          //mp 
           
          //mp 
           
          //mp 
          break;	
      }
      //right_loading_station_waypoint
      switch (m_secondDestination) {
        case Robot.second_cargo_cross_front:
          //mp 
           
          //mp 
           
          //mp 
          break;
        case Robot.second_cargo_near:
          //mp 
           
          //mp 
           
          //mp 
          break;
        case Robot.second_cargo_mid:
          //mp 
           
          //mp 
           
          //mp 
          break;
        case Robot.second_cargo_far:
          //mp 
           
          //mp 
           
          //mp 
          break;
        
      }
      break;
    case "Left Level 2":
      switch (m_firstDestination) {
        case Robot.first_cargo_front:
          // addSequential(new GoLeftCargoFrontToLeftLoadingBay());
          break;
        case Robot.first_cargo_near:
          //mp 
           
          //mp 
           
          //mp 
          break;
        case Robot.first_cargo_mid:
          //mp 
           
          //mp 
           
          //mp 
          break;
        case Robot.first_cargo_far:
          //mp 
           
          //mp 
           
          //mp 
          break;
      }
      //left_loading_station_waypoint
      switch (m_secondDestination) {
        case Robot.second_cargo_cross_front:
          //mp 
           
          //mp 
           
          //mp 
          break;
        case Robot.second_cargo_near:
          
          // addSequential(new GoLeftLoadingToLeftCargoNear());
          
          break;
        case Robot.second_cargo_mid:
          //mp 
           
          //mp 
           
          //mp 
          break;
        case Robot.second_cargo_far:
          //mp 
           
          //mp 
           
          //mp 
          break;
      }
      break;
      case "Left Level 1":
      switch (m_firstDestination) {
        case Robot.first_cargo_front:
          addSequential(new GoLeftLevel1ToLeftCargoFront());
          // addSequential(new GoLeftCargoFrontToLeftLoadingBay());
          break;
        case Robot.first_cargo_near:
          //mp 
           
          //mp 
           
          //mp 
          break;
        case Robot.first_cargo_mid:
          //mp 
           
          //mp 
           
          //mp 
          break;
        case Robot.first_cargo_far:
          //mp 
           
          //mp 
           
          //mp 
          break;	
      }
      switch (m_secondDestination) {
        case Robot.second_cargo_cross_front:
          //mp 
           
          //mp 
           
          //mp 
          break;
        case Robot.second_cargo_near:
          // addSequential(new GoLeftLoadingToLeftCargoNear());
          break;
        case Robot.second_cargo_mid:
          //mp 
           
          //mp 
           
          //mp 
          break;
        case Robot.second_cargo_far:
          //mp 
           
          //mp 
           
          //mp 
          break;
        
      }
      break;
      case "Right Level 1":
      switch (m_firstDestination) {
        case Robot.first_cargo_front:
          System.out.println("Right Level 1 to Right Cargo Front");
          addSequential(new GoRightLevel1ToRightCargoFront());
          // addSequential(new GoRightCargoFrontToRightLoadingBay());
          break;
        case Robot.first_cargo_near:
          //mp 
           
          //mp 
           
          //mp 
          break;
        case Robot.first_cargo_mid:
          //mp 
           
          //mp 
           
          //mp 
          break;
        case Robot.first_cargo_far:
          //mp 
           
          //mp 
           
          //mp 
          break;	
      }
      //right_loading_station_waypoint
      switch (m_secondDestination) {
        case Robot.second_cargo_cross_front:
          //mp 
           
          //mp 
           
          //mp 
          break;
        case Robot.second_cargo_near:
          //mp 
          
          //mp 
         
          //mp 
          break;
        case Robot.second_cargo_mid:
          //mp 
           
          //mp 
           
          //mp 
          break;
        case Robot.second_cargo_far:
          //mp 
           
          //mp 
           
          //mp 
          break;
        
      }
      break;
  }
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

  }
}
