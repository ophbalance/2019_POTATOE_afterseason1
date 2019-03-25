/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.*;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  public Joystick _driver = new Joystick(RobotMap.OI_DRIVER_CONTROLLER); 
  public Joystick _operator = new Joystick(RobotMap.OI_OP_CONTROLLER);
  public Joystick _squishy = new Joystick(RobotMap.OI_SQUISHY);

  /*public Button LADfrontClimb = new JoystickButton(_operator, 2);
  public Button LADrearClimb = new JoystickButton(_operator, 3);
  public Button LADfrontDrop = new JoystickButton(_operator, 5);
  public Button LADrearDrop = new JoystickButton(_operator, 4);
  public Button LADallClimb = new JoystickButton(_operator, 6);
  public Button LADDriveFwd = new JoystickButton(_operator, 1);
  public Button SquishyEndoIn = new JoystickButton(_operator, 7);
  public Button SquishyEndoOut = new JoystickButton(_operator, 8);
  public Button SquishyClawOpen = new JoystickButton(_operator, 9);
  public Button SquishyClawClose = new JoystickButton(_operator, 10);
*/
public Button LADDriveFwd = new JoystickButton(_operator, 1);
public Button LADUpdateAll = new JoystickButton(_operator, 4);

//NEEDS TESTING, not sure button map
  public Button SquishyTiltUp = new JoystickButton(_squishy, 4);
  public Button SquishyTiltDown = new JoystickButton(_squishy, 1);
  public Button SquishyClawOpen = new JoystickButton(_squishy, 5);
  public Button SquishyClawClose = new JoystickButton(_squishy, 6);
//^^^^^^^^



  public OI () {
//GOOD
/*
    LADfrontClimb.whileHeld(new LADFrontUpdate(-RobotMap.FRONT_SPEED));
    LADrearClimb.whileHeld(new LADRearUpdate(-RobotMap.REAR_SPEED));
    
    LADfrontDrop.whileHeld(new LADFrontUpdate(RobotMap.FRONT_SPEED));
    LADrearDrop.whileHeld(new LADRearUpdate(RobotMap.REAR_SPEED));
    */
    LADDriveFwd.whileHeld(new LADUpdateDrive(RobotMap.DRIVE_SPEED));
    LADUpdateAll.whileHeld(new LADAllLift(RobotMap.FRONT_SPEED,RobotMap.REAR_SPEED));

//NEEDS TESTING
/*
    LADallClimb.whileHeld(new LADAllLift(RobotMap.FRONT_SPEED,RobotMap.REAR_SPEED));    

    SquishyEndoIn.whileHeld(new squishyIntake(RobotMap.INTAKE_SPEED));
    SquishyEndoOut.whileHeld(new squishyExhaust(RobotMap.EXHAUST_SPEED));
    SquishyClawOpen.whileHeld(new squishyClawOpen());
    SquishyClawClose.whileHeld(new squishyClawClose());
*/
    SquishyTiltUp.whileHeld(new squishyTiltUpdate(-RobotMap.TILT_SPEED));
    SquishyTiltDown.whileHeld(new squishyTiltUpdate(RobotMap.TILT_SPEED));
    SquishyClawClose.whileHeld(new squishyClawClose(-RobotMap.TILT_SPEED));
    SquishyClawOpen.whileHeld(new squishyClawOpen(RobotMap.TILT_SPEED));
    

    //RunClimbDown.whileHeld(new climbdown());    
    //RunDriveBack.whileHeld(new climbdriveback());
    //UpFront.whileHeld(new climbupdown());
    //UpBak.whileHeld(new climbbackup());
    
    
  }
}
