/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.VictorSP;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.*;
/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Squishy extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
    Spark leftendo = new Spark(RobotMap.INTAKE_LEFT);
    Spark rightendo = new Spark(RobotMap.INTAKE_RIGHT);
    VictorSP open = new VictorSP(RobotMap.OPEN);
    Spark tilt = new Spark(RobotMap.TILT);

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
     setDefaultCommand(new squishyAxis());
  }


    public void suckin(double p_val) {
      // Update motor speed to passed in value
      leftendo.set(-p_val);
      rightendo.set(p_val);
    }

    public void intakeUpdate(double p_val) {
      // Update motor speed to passed in value
      leftendo.set(-p_val);
      rightendo.set(p_val);
    }

    public void blowout(double p_val) {
      // Update motor speed to passed in value
      leftendo.set(p_val);
      rightendo.set(-p_val);
    }

    public void tilt() {
      // Update motor speed to passed in value
      //tilt.set(p_val);
    }

    public void tiltupdate(double p_val) {
      // Update motor speed to passed in value
      tilt.set(p_val);
    }

    public void tiltup(double p_val) {
      // Update motor speed to passed in value
      tilt.set(p_val);
    }

    public void tiltdown(double p_val) {
      // Update motor speed to passed in value
      tilt.set(-p_val);
    }

    public void openup(double p_val) {
      // Update motor speed to passed in value
      open.set(p_val);
    }
    

    public void closeup(double p_val) {
      // Update motor speed to passed in value
      open.set(-p_val);
    }
}
