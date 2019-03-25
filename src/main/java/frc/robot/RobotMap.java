/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  //Drive Train
  public static final int DRIVETRAIN_LEFT_FRONT = 11;  //CAN
	public static final int DRIVETRAIN_LEFT_BACK = 13;   //CAN
	public static final int DRIVETRAIN_RIGHT_FRONT = 10; //CAN
  public static final int DRIVETRAIN_RIGHT_BACK = 12;  //CAN
  
  //LAD
  public static final int LAD_FRONT = 1;        //PWM
  public static final int LAD_BACK = 0;         //PWM
  public static final int LAD_DRIVE = 2;        //PWM
  public static final double FRONT_SPEED = .5;  //const
  public static final double REAR_SPEED = .5;   //const
  public static final double DRIVE_SPEED = 1;   //const
  
  //Elevator
  public static final int LIFT = 3;             //PWM
  public static final double LIFT_SPEED = .6;   //const

  //Squishy
  public static final int INTAKE_LEFT = 7;        //PWM
  public static final int INTAKE_RIGHT = 6;       //PWM
  public static final int OPEN = 5;               //PWM
  public static final int TILT = 4;               //PWM
  public static final double INTAKE_SPEED = .6;   //const
  public static final double EXHAUST_SPEED = .6;  //const
  public static final double TILT_SPEED = .6;   //const
  public static final double MOVE_JAWS = .5;      //const

  // Joysticks
	public static final int OI_DRIVER_CONTROLLER = 0; //joy1
  public static final int OI_OP_CONTROLLER = 1;     //joy2
  public static final int OI_SQUISHY = 2;           //joy3
  
	public static final int DRIVE_JOY_FORWARD = 1;    //JOY_axis
	public static final int DRIVE_JOY_TURN = 3;       //JOY_axis

	//public static final int OP_LEFTSTICK = 1;     //JOY_axis
  //public static final int OP_RIGHTSTICK = 5;    //JOY_axis

  public static final int OP_XBOX_LEFTSTICK = 1;    //JOY_axis
  public static final int OP_XBOX_RIGHTSTICK = 5;    //JOY_axis

  public static final int SQUISHY_XBOX_LEFTSTICK = 1;    //JOY_axis
  public static final int SQUISHY_XBOX_RIGHTSTICK = 5;    //JOY_axis

}
