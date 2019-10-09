/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj.DoubleSolenoid;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Pnuemy extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  DoubleSolenoid testSol = new DoubleSolenoid(0, 1);
  
  //exampleDouble.set(DoubleSolenoid.Value.kOff);
  //exampleDouble.set(DoubleSolenoid.Value.kForward);
  //exampleDouble.set(DoubleSolenoid.Value.kReverse);

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
     setDefaultCommand(new SolOpen());
  }


    public void close() {
      // Update motor speed to passed in value
      testSol.set(DoubleSolenoid.Value.kReverse);
    }

    public void open() {
      // Update motor speed to passed in value
      testSol.set(DoubleSolenoid.Value.kForward);
    }
}
