/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.POVButton;
import edu.wpi.first.wpilibj.buttons.Trigger;
import frc.robot.commands.*;
import frc.robot.commands.drivetrain.BobDrive;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button number it is.    
  public Joystick driveStick = new Joystick(0);                       // Drive
    Button button1 = new JoystickButton(driveStick, 1);               // Hatch Panel Release (Trigger)
	  Button button2 = new JoystickButton(driveStick, 2);               // 
	  Button button3 = new JoystickButton(driveStick, 3);               // 
	  Button button4 = new JoystickButton(driveStick, 4);               // 
	  Button button5 = new JoystickButton(driveStick, 5);               // 
	  Button button6 = new JoystickButton(driveStick, 6);               // 
	  Button button7 = new JoystickButton(driveStick, 7);               // 
	  Button button8 = new JoystickButton(driveStick, 8);               // 
	  Button button9 = new JoystickButton(driveStick, 9);               // 
	  Button button10 = new JoystickButton(driveStick, 10);             // 
	  Button button11 = new JoystickButton(driveStick, 11);             // 
    Button button12 = new JoystickButton(driveStick, 12);             // 

  public Joystick testStick = new Joystick(2);
  public Joystick xboxController = new Joystick(1);                   //  
    Button xboxA = new JoystickButton(xboxController, 1);      // A lvl 1 rocket cargo 
    Button xboxB = new JoystickButton(xboxController, 2);      // B lvl 2 rocket
    Button xboxX = new JoystickButton(xboxController, 3);      // X lvl 2 rocket cargo
    Button xboxY = new JoystickButton(xboxController, 4);      // Y lvl 3 rocket cargo
    Button xboxLB = new JoystickButton(xboxController, 5);     // LEFTBUMPERS Extension Rail Extend or Retract
    Button xboxRB = new JoystickButton(xboxController, 6);     // RIGHTBUMPERS CARGO EJECT
    Button xboxS = new JoystickButton(xboxController, 7);      // SELECT/BACK    together 7 & 8 climbzsczsc
    Button xboxO = new JoystickButton(xboxController, 8);      // OPTIONS/START
    Button xboxLS = new JoystickButton(xboxController, 9);     // LEFTJOYSTICK Drive with Cargo
    Button xboxRS = new JoystickButton(xboxController, 10);   // RIGHTJOYSTICK CargoShip Cargo
    POVButton xboxDPADTOP = new POVButton(xboxController, 0); // lvl 3 rocket hatch
    POVButton xboxDPADRIGHT = new POVButton(xboxController, 90); // lvl 2 rocket hatch
    POVButton xboxDPADBOTTOM = new POVButton(xboxController, 180); // lvl hatch
    POVButton xboxDPADLEFT = new POVButton(xboxController, 270); // lvl 2 rocket hatch
   
    Trigger xboxLJS = new Trigger(){
      @Override
      public boolean get() {
        return xboxController.getRawAxis(0)<RobotMap.DEADBAND_NEGATIVE || xboxController.getRawAxis(0)>RobotMap.DEADBAND_POSITIVE || xboxController.getRawAxis(1)<RobotMap.DEADBAND_NEGATIVE || xboxController.getRawAxis(1)>RobotMap.DEADBAND_POSITIVE;
        }
    };
   
    Trigger xboxRJS = new Trigger(){
      @Override
      public boolean get() {
        return xboxController.getRawAxis(4)<RobotMap.DEADBAND_NEGATIVE || xboxController.getRawAxis(4)>RobotMap.DEADBAND_POSITIVE || xboxController.getRawAxis(5)<RobotMap.DEADBAND_NEGATIVE || xboxController.getRawAxis(5)>RobotMap.DEADBAND_POSITIVE;
        }
    };
   
    Trigger xboxClimbLevelTwo = new Trigger(){
      @Override
      public boolean get() {
        return DriverStation.getInstance().isOperatorControl() && driveStick.getRawButton(7) && driveStick.getRawButton(8);
      
        // return DriverStation.getInstance().getMatchTime() <= RobotMap.TIME_BEFORE_CLIMB && DriverStation.getInstance().isOperatorControl() && driveStick.getRawButton(7) && driveStick.getRawButton(8);
      }
    };
    
    Trigger xboxClimbLevelThree = new Trigger(){
      @Override
      public boolean get() {
        return DriverStation.getInstance().isOperatorControl() && xboxController.getRawButton(7) && xboxController.getRawButton(8);
      
        // return DriverStation.getInstance().getMatchTime() <= RobotMap.TIME_BEFORE_CLIMB && DriverStation.getInstance().isOperatorControl() && xboxController.getRawButton(7) && xboxController.getRawButton(8);
      }
    };
    
    Trigger resetEncoders = new Trigger(){
      @Override
      public boolean get() {
        return DriverStation.getInstance().isOperatorControl() && driveStick.getRawButton(11) && driveStick.getRawButton(12);
      
        // return DriverStation.getInstance().getMatchTime() <= RobotMap.TIME_BEFORE_CLIMB && DriverStation.getInstance().isOperatorControl() && driveStick.getRawButton(5) && driveStick.getRawButton(6);
      }
    };
    Trigger cancelClimb = new Trigger(){
      @Override
      public boolean get() {
        return DriverStation.getInstance().isOperatorControl() && driveStick.getRawButton(3) && driveStick.getRawButton(4);
      
        // return DriverStation.getInstance().getMatchTime() <= RobotMap.TIME_BEFORE_CLIMB && DriverStation.getInstance().isOperatorControl() && driveStick.getRawButton(3) && driveStick.getRawButton(4);
      }
    };
    
    Trigger cargoIntake = new Trigger(){
    
      @Override
      public boolean get() {
        return xboxController.getRawAxis(3) > RobotMap.DEADBAND_POSITIVE;
      }
    };

    Trigger joystickDrive = new Trigger(){

      @Override
      public boolean get() {
        return driveStick.getRawAxis(3) < 0;
      }
    };

    Trigger autonomousDrive = new Trigger(){

      @Override
      public boolean get() {
        return driveStick.getRawAxis(3) > 0 && DriverStation.getInstance().isAutonomous() && DriverStation.getInstance().isEnabled();
      }
    };
    
 Trigger startAutoSequenceSelector = new Trigger(){

  @Override
  public boolean get() {
    return DriverStation.getInstance().isAutonomous() && DriverStation.getInstance().isEnabled() && DriverStation.getInstance().getMatchTime() >= 149;
  }
};



    /*Trigger returnToBobDrive = new Trigger(){

      @Override
      public boolean get() {
        return Robot.drivetrain.isClosedLoopFailure;
      }whenActive
    };*/

    Trigger disableElevatorTop = new Trigger(){
    
      @Override
      public boolean get() {
        return xboxController.getRawAxis(2) > RobotMap.DISABLE_ELEVATOR_TOP_TRIGGER_DISTANCE;
        //return (DriverStation.getInstance().getMatchTime() <= RobotMap.TIME_BEFORE_CLIMB) && (xboxController.getRawAxis(2) > RobotMap.DISABLE_ELEVATOR_TOP_TRIGGER_DISTANCE);
      }
    };
    
  public OI (){
    xboxA.whenPressed(new ElevatorMove(RobotMap.LEVEL_ONE_CARGO_ROCKET));
    xboxB.whenPressed(new ElevatorMove(RobotMap.LEVEL_TWO_CARGO));
    xboxX.whenPressed(new ElevatorMove(RobotMap.LEVEL_TWO_CARGO));
    xboxY.whenPressed(new ElevatorMove(RobotMap.LEVEL_THREE_CARGO));
    xboxDPADTOP.whenPressed(new ElevatorMove(RobotMap.LEVEL_THREE_HATCH));
    xboxDPADRIGHT.whenPressed(new ElevatorMove(RobotMap.LEVEL_TWO_HATCH));
    xboxDPADBOTTOM.whenPressed(new ElevatorMove(RobotMap.LEVEL_ONE_HATCH));
    xboxDPADLEFT.whenPressed(new ElevatorMove(RobotMap.LEVEL_TWO_HATCH));
    xboxLS.whenPressed(new ElevatorMove(RobotMap.LEVEL_DRIVE_WITH_CARGO));
    xboxRS.whenPressed(new ElevatorMove(RobotMap.LEVEL_ONE_CARGO_CARGOSHIP));
    xboxLJS.whenActive(new ElevatorMove(RobotMap.LEVEL_DRIVE_WITH_CARGO));
    xboxRJS.whenActive(new ElevatorMove(RobotMap.LEVEL_ONE_CARGO_CARGOSHIP));
    button1.whenPressed(new HatchPanelRelease());
    //button1.whileHeld(new HatchPanelRelease());
    button1.whenReleased(new HatchPanelHold());
 
    cancelClimb.whenActive(new ClimbCancel());
    xboxRB.whenPressed(new CargoEject());
    xboxClimbLevelTwo.whenActive(new ClimbGroupMain(RobotMap.CLIMB_LEVEL_TWO));
    xboxClimbLevelThree.whenActive(new ClimbGroupMain(RobotMap.CLIMB_LEVEL_THREE));
    cargoIntake.whenActive(new CargoIntake());
    xboxLB.whenPressed(new ExtensionRailCommand());
    resetEncoders.whenActive(new ResetCommandGroup());
    disableElevatorTop.whenActive(new DisableElevatorTopDelay());
    
    joystickDrive.whenActive(new BobDrive());
      //returnToBobDrive.whenActive(new BobDrive());
//    autonomousDrive.whenActive(new AutoSequenceSelector());

    //button11.whileHeld(new LimeLightTrackToArea(true));  //on target
    button2.whileHeld(new LimeLightTrackPIDToAreaLS());      //185mm off target
    //button2.whileHeld(new LimeLightTrackPIDToArea());      //185mm off target
    
    //button3.whileHeld(new LimeLightTrackToArea());       //185mm off target
    //button3.whileHeld(new LimeLightTrackPIDToArea(true));    //on target
    //button4.whileHeld(new LimeLightTrackToArea(true));       //on target
    //button8.whenPressed(new GoLeftLevel1ToLeftCargoFront());
    // button9.whenPressed(new GoRightLevel1ToRightCargoFront());
    // button9.whenPressed(new AutoSequenceSelector());

    //button7.whenPressed(new GoLeftCargoFrontToLeftLoadingBay());
	//	button7.whenPressed(new GoRightCargoFrontToRightLoadingBay());
    
    //button10.whenPressed(new GoLeftLoadingToLeftRocketFar());
    //button10.whenPressed(new GoRightLoadingToRightRocketFar());
    
    //button9.whenPressed(new GoLeftLoadingToLeftRocketNear());
    //button9.whenPressed(new GoRightLoadingToRightRocketNear());
    
    
    
    //button4.whenPressed(new GoLeftLoadingToLeftCargoFront());
    //button11.whenPressed(new GoLeftLoadingToLeftCargoNear());
    //button8.whenPressed(new GoLeftLevel2ToLeftCargoFront());
		//button12.whileHeld(new KnightRider());
    //button12.whileHeld(new KnightRider(false, 0, 4.2));//4.2));
    //button9.whenPressed(new FollowArc("/home/lvuser/motionprofiles/LeftLoadingBayToLeftReversedFromLoadingBay.dat",50,true,true));
		//button10.whenPressed(new FollowArc("/home/lvuser/motionprofiles/LeftLoadingBayToLeftReversedFromLoadingBay.dat"));
		//button8.whenPressed(new FollowArc("/home/lvuser/motionprofiles/LeftReversedFromLoadingBayToLeftRocketNear.dat",50,true,false));
		//button7.whenPressed(new FollowArc("/home/lvuser/motionprofiles/LeftReversedFromLoadingBayToLeftRocketNear.dat",true,true));
		//button9.whenPressed(new GoLeftLoadingToCargo3Reverse());
		//button10.whenPressed(new GoLoadingToCargo1());
		//button9.whenPressed(new GoLoadingToCargo1Reverse());
		//button4.whenPressed(new GoLeftLoadingToCargo2());
		//button7.whenPressed(new GoLeftLoadingToCargoFront());
		//button5.whenPressed(new GoLeftCargo3ToLoading());
    //Autonomous Mode Limielight Track (goes all the way)
    // button5.whileHeld(new LimeLightTrackToArea(true));
    //Teleop Mode Limielight Track (stops 185mm short of targets)
    //button2.whileHeld(new LimeLightTrack());
    //button6.whenPressed(new ResetGyro());
    //button4.whenPressed(new ResetGyro(270));


   } 
  public Joystick driveStick(){
    return driveStick(); 
   }
   public Joystick testStick(){
    return testStick(); 
   }
    

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());
}
