/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.OI;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.Timer;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;


  
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.*;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static Squishy squishy;
  public static LAD lad;
  public static Elevator elevator;
  public static DriveTrain drive;
  public static AHRS ahrs;
  public double startTime=0;
  WPI_TalonSRX _leftMaster = new WPI_TalonSRX(11);
    WPI_TalonSRX _rightMaster = new WPI_TalonSRX(10);
    WPI_VictorSPX  _leftFollow = new WPI_VictorSPX (13);
    WPI_VictorSPX  _rightFollow = new WPI_VictorSPX (12);
    DifferentialDrive _drive = new DifferentialDrive(_leftMaster, _rightMaster);
  

  public static OI m_oi;

  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    
    //try {
			/***********************************************************************
			 * navX-MXP:
			 * - Communication via RoboRIO MXP (SPI, I2C, TTL UART) and USB.            
			 * - See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
			 * 
			 * navX-Micro:
			 * - Communication via I2C (RoboRIO MXP or Onboard) and USB.
			 * - See http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
			 * 
			 * Multiple navX-model devices on a single robot are supported.
			 ************************************************************************/
      //      ahrs = new AHRS(SPI.Port.kMXP); 
        //} catch (RuntimeException ex ) {
          //  DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
       // }
        
    squishy = new Squishy();
    lad =  new LAD();
    elevator = new Elevator();
    /* COMMENT ME TO USE DRIVE IN THIS FILE */
    drive = new DriveTrain();
    /* */
    _leftMaster.configFactoryDefault();
        _rightMaster.configFactoryDefault();
        _leftFollow.configFactoryDefault();
        _rightFollow.configFactoryDefault();
        
        _leftFollow.follow(_leftMaster);
        _rightFollow.follow(_rightMaster);
        
        _leftMaster.setInverted(false); // <<<<<< Adjust this until robot drives forward when stick is forward
        _rightMaster.setInverted(true); // <<<<<< Adjust this until robot drives forward when stick is forward
        _leftFollow.setInverted(InvertType.FollowMaster);
        _rightFollow.setInverted(InvertType.FollowMaster);
        _drive.setRightSideInverted(false); // do not change this

    m_oi = new OI();
    //m_chooser.setDefaultOption("Default Auto", new ExampleCommand());
    // chooser.addOption("My Auto", new MyAutoCommand());
    //SmartDashboard.putData("Auto mode", m_chooser);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_chooser.getSelected();
    startTime = Timer.getFPGATimestamp();

    


    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();

    //TIMED TEST
    /*
    if (startTime < 4){
      _drive.arcadeDrive(-.4, 0);
    }
    */
    double forward = 1 * m_oi._driver.getY();
    double turn = m_oi._driver.getTwist();
    //ouble liftup = m_oi._operator.getY();
    //double driveLift = m_oi._game2.getRawAxis(5);
    forward = Deadband(forward);
    turn = Deadband(turn);
    //_leftMaster.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, turn*.55);
    //_rightMaster.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, -turn*.55);
    _drive.arcadeDrive(-forward, turn);
    //                                                                                                                                                                                                                                                                                                                                                                                                    teleopPeriodic();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

    /*
    _leftMaster.configFactoryDefault();
        _rightMaster.configFactoryDefault();
        _leftFollow.configFactoryDefault();
        _rightFollow.configFactoryDefault();
        
        _leftFollow.follow(_leftMaster);
        _rightFollow.follow(_rightMaster);
        
        _leftMaster.setInverted(false); // <<<<<< Adjust this until robot drives forward when stick is forward
        _rightMaster.setInverted(true); // <<<<<< Adjust this until robot drives forward when stick is forward
        _leftFollow.setInverted(InvertType.FollowMaster);
        _rightFollow.setInverted(InvertType.FollowMaster);
        _drive.setRightSideInverted(false); // do not change this
*/
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();

    
    double forward = 1 * m_oi._driver.getY();
    double turn = m_oi._driver.getTwist();
    //ouble liftup = m_oi._operator.getY();
    //double driveLift = m_oi._game2.getRawAxis(5);
    forward = Deadband(forward);
    turn = Deadband(turn);
    //_leftMaster.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, turn*.35);
    //_rightMaster.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, -turn*.35);
    _drive.arcadeDrive(-forward, turn);
    
  }

  /** Deadband 5 percent, used on the gamepad */
double Deadband(double value) {
  /* Upper deadband */
  if (value >= +0.35) 
      return value;
  
  /* Lower deadband */
  if (value <= -0.35)
      return value;
  
  /* Outside deadband */
  return 0;
}

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
