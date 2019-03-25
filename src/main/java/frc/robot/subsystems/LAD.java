/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.Spark;

import frc.robot.commands.*;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.VictorSP;


/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class LAD extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  
  static final double kOffBalanceAngleThresholdDegrees = 10;
  static final double kOonBalanceAngleThresholdDegrees  = 5;
  boolean autoBalanceXMode=false;
  boolean autoBalanceYMode=false;
  VictorSP driveMotor = new VictorSP(RobotMap.LAD_DRIVE);
  VictorSP f_motor = new VictorSP(RobotMap.LAD_FRONT);
  VictorSP r_motor = new VictorSP(RobotMap.LAD_BACK);
  //Spark f_motor = new Spark(RobotMap.LAD_FRONT);
  //Spark r_motor = new Spark(RobotMap.LAD_BACK);


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
     setDefaultCommand(new LADAxis());
     
  }


  public void updateFront(double p_val) {
    // Update motor speed to passed in value
    f_motor.set(p_val);
    System.out.println("front motor");
    System.out.println(f_motor);
  }

  public void updateRear(double p_val) {
    // Update motor speed to passed in value
    r_motor.set(p_val);
    System.out.println("rear motor");
    System.out.println(r_motor);
  }

  public void updateAxis() {
    // Update motor speed to passed in value
    double input = Robot.m_oi._operator.getRawAxis(RobotMap.OP_XBOX_LEFTSTICK);
    double input2 = Robot.m_oi._operator.getRawAxis(RobotMap.OP_XBOX_RIGHTSTICK);

    updateFront(-input);
    updateRear(input2);
    System.out.println("LAD axis up/down");
    System.out.println(input);
    System.out.println(input2);
  }

  public void updateDriveMotor(double p_val) {
    // Update motor speed to passed in value
    driveMotor.set(p_val);
    System.out.println("Drive Motor");
    System.out.println(p_val);
  }

  public void updateAll(double p_front, double p_rear) {
    // Update motor speed to passed in value
    double pitchAngleDegrees    = Robot.ahrs.getPitch();
    if ( !autoBalanceXMode && 
        (Math.abs(pitchAngleDegrees) >= 
        Math.abs(kOffBalanceAngleThresholdDegrees))) {
      autoBalanceXMode = true;
    }
    else if ( autoBalanceXMode && 
            (Math.abs(pitchAngleDegrees) <= 
              Math.abs(kOonBalanceAngleThresholdDegrees))) {
      autoBalanceXMode = false;
    }
    if ( autoBalanceXMode ) {
        double pitchAngleRadians = pitchAngleDegrees * (Math.PI / 180.0);
        //p_front = Math.sin(pitchAngleRadians) * -1;
        //p_rear = Math.sin(pitchAngleRadians) * -1;
        System.out.println(Math.sin(pitchAngleRadians) * -1);
        //System.out.println(Math.sin(pitchAngleRadians) * -1);
    }
    //updateFront(p_front);
    //updateRear(p_rear);
  }
}
