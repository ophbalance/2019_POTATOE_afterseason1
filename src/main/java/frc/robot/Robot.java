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
//import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;


  
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.TimedRobot;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;


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
  //  public static AHRS ahrs;
    boolean autoBalanceXMode;
    boolean autoBalanceYMode;
    static final double kOffBalanceAngleThresholdDegrees = 10;
    static final double kOonBalanceAngleThresholdDegrees  = 5;
    public double startTime=0;
    WPI_TalonSRX _leftMaster = new WPI_TalonSRX(11);
    WPI_TalonSRX _rightMaster = new WPI_TalonSRX(10);
    WPI_VictorSPX  _leftFollow = new WPI_VictorSPX (13);
    WPI_VictorSPX  _rightFollow = new WPI_VictorSPX (12);
    DifferentialDrive _drive = new DifferentialDrive(_leftMaster, _rightMaster);
    private boolean driverVision, tapeVision, cargoVision, cargoSeen, tapeSeen   ;
    private NetworkTableEntry tapeDetected, cargoDetected, tapeYaw, cargoYaw, videoTimestamp, driveWanted,tapeWanted,cargoWanted;
    private double targetAngle;
    NetworkTableInstance instance;
    NetworkTable chickenVision;

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
    /*try {
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
        /*    ahrs = new AHRS(SPI.Port.kMXP); 
        } catch (RuntimeException ex ) {
            DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
        }*/
        System.out.println("Starting Navx");
        instance = NetworkTableInstance.getDefault();
 
        chickenVision = instance.getTable("ChickenVision");
 
        tapeDetected = chickenVision.getEntry("tapeDetected");
        cargoDetected = chickenVision.getEntry("cargoDetected");
        tapeYaw = chickenVision.getEntry("tapeYaw");
        cargoYaw = chickenVision.getEntry("cargoYaw");
 
        driveWanted = chickenVision.getEntry("Driver");
        tapeWanted = chickenVision.getEntry("Tape");
        cargoWanted = chickenVision.getEntry("Cargo");
 
        videoTimestamp = chickenVision.getEntry("VideoTimestamp");
 
        tapeVision = cargoVision = false;
        driverVision = true;
        targetAngle = 0;

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

    double kP = 1.2;
    double frontRate            = .4;
    double pitchRate            = 0;
    double rearRate            = .6;
    double yAxisRate            = m_oi._operator.getY();
    //double pitchAngleDegrees    = ahrs.getPitch();
    //Sdouble rollAngleDegrees     = ahrs.getRoll();
    boolean tapeDesired = m_oi._driver.getRawButton(1);
    boolean cargoDesired = m_oi._driver.getRawButton(2);

    double forward = 1 * m_oi._driver.getY();
    double turn = m_oi._driver.getTwist();
    double convertFwd = 0;
    double convertTrn = 0;

    //This code is all about vision tracking!!!
    // If button 1 is pressed, then it will track cargo
    if (cargoDesired) {
 
      driveWanted.setBoolean(false);
      tapeWanted.setBoolean(false);
      cargoWanted.setBoolean(true);
      cargoSeen = cargoDetected.getBoolean(false);

      if (cargoSeen)
          targetAngle = cargoYaw.getDouble(0);
      else
          targetAngle = 0;

    } else if (tapeDesired) {


        driveWanted.setBoolean(false);
        tapeWanted.setBoolean(true);
        cargoWanted.setBoolean(false);
        // Checks if vision sees cargo or vision targets. This may not get called unless
        // cargo vision detected
        tapeSeen = tapeDetected.getBoolean(false);

        if (tapeSeen)
            targetAngle = tapeYaw.getDouble(0);
        else
            targetAngle = 0;

    } else {


        driveWanted.setBoolean(true);
        tapeWanted.setBoolean(false);
        cargoWanted.setBoolean(false);

        targetAngle = 0;

    }
    //This code is all about vision tracking!!!


    

    convertFwd = Math.pow(forward, 2);
      if(forward < 0) forward=convertFwd * -1;
    convertTrn=Deadband(turn);
    convertTrn = Math.pow(convertTrn, 2);
      if(turn < 0) turn=convertTrn * -1;
    if(turn > .75) turn=.75;
    if(turn < -.75) turn=-.75;
    double output = limitOutput(-kP * targetAngle, 0.4);

    //So, hey.  We pressed a button.  So turn to a target.
    if (cargoDesired || tapeDesired)    
      _drive.arcadeDrive(-forward, -output);
    else  //Or not, cuz, that's like fine too
       _drive.arcadeDrive(-forward, turn);
      
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

    double kP = 1.2;
    double frontRate            = .4;
    double pitchRate            = 0;
    double rearRate            = .6;
    double yAxisRate            = m_oi._operator.getY();
    //double pitchAngleDegrees    = ahrs.getPitch();
    //Sdouble rollAngleDegrees     = ahrs.getRoll();
    boolean tapeDesired = m_oi._driver.getRawButton(1);
    boolean cargoDesired = m_oi._driver.getRawButton(2);

    double forward = 1 * m_oi._driver.getY();
    double turn = m_oi._driver.getTwist();
    double convertFwd = 0;
    double convertTrn = 0;

    //This code is all about vision tracking!!!
    // If button 1 is pressed, then it will track cargo
    if (cargoDesired) {
 
      driveWanted.setBoolean(false);
      tapeWanted.setBoolean(false);
      cargoWanted.setBoolean(true);
      cargoSeen = cargoDetected.getBoolean(false);

      if (cargoSeen)
          targetAngle = cargoYaw.getDouble(0);
      else
          targetAngle = 0;

    } else if (tapeDesired) {


        driveWanted.setBoolean(false);
        tapeWanted.setBoolean(true);
        cargoWanted.setBoolean(false);
        // Checks if vision sees cargo or vision targets. This may not get called unless
        // cargo vision detected
        tapeSeen = tapeDetected.getBoolean(false);

        if (tapeSeen)
            targetAngle = tapeYaw.getDouble(0);
        else
            targetAngle = 0;

    } else {


        driveWanted.setBoolean(true);
        tapeWanted.setBoolean(false);
        cargoWanted.setBoolean(false);

        targetAngle = 0;

    }
    //This code is all about vision tracking!!!


    

    convertFwd = Math.pow(forward, 2);
      if(forward < 0) forward=convertFwd * -1;
    convertTrn=Deadband(turn);
    convertTrn = Math.pow(convertTrn, 2);
      if(turn < 0) turn=convertTrn * -1;
    if(turn > .75) turn=.75;
    if(turn < -.75) turn=-.75;
    double output = limitOutput(-kP * targetAngle, 0.4);

    //So, hey.  We pressed a button.  So turn to a target.
    if (cargoDesired || tapeDesired)    
      _drive.arcadeDrive(-forward, -output);
    else  //Or not, cuz, that's like fine too
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

//Do a nifty thing to limit power applied to turning
public double limitOutput(double number, double maxOutput) {
  if (number > 1.0) {
      number = 1.0;
  }
  if (number < -1.0) {
      number = -1.0;
  }

  if (number > maxOutput) {
      return maxOutput;
  }
  if (number < -maxOutput) {
      return -maxOutput;
  }

  return number;
}

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
