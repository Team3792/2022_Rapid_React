// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.IntakeCmd;
import frc.robot.subsystems.AAPowerDistribution;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FeedSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
// import edu.wpi.first.cscore.UsbCamera;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
import frc.robot.subsystems.ShooterSubsystem;
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  // private PDHSubsystem m_PDH = new PDHSubsystem();
  private IntakeSubsystem m_intake = new IntakeSubsystem();
  private FeedSubsystem m_feeder = new FeedSubsystem();
  private ElevatorSubsystem elevator = new ElevatorSubsystem();
  private DriveSubsystem drive = new DriveSubsystem();




  public final WPI_TalonFX rightLead = new WPI_TalonFX(Constants.MotorID.kRightDriveLead);
  public final WPI_TalonFX rightFollow = new WPI_TalonFX(Constants.MotorID.kRightDriveFollow);
  public final WPI_TalonFX leftLead = new WPI_TalonFX(Constants.MotorID.kLeftDriveLead);
  public final WPI_TalonFX leftFollow = new WPI_TalonFX(Constants.MotorID.kLeftDriveFollow);
  public final WPI_TalonFX shooter = new WPI_TalonFX(Constants.MotorID.kShootMotor);

  





  //Path weaver stuff



  // @Override
	// public void simulationInit() {
	// 	PhysicsSim.getInstance().addTalonFX(elevator.leftElevatorMotor, 5, 15596);
	// 	PhysicsSim.getInstance().addTalonFX(elevator.rightElevatorMotor, 5, 15596);
  // }

	// @Override
	// public void simulationPeriodic() {
	// 	PhysicsSim.getInstance().run();
	// }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.

    





    m_robotContainer = new RobotContainer();
    // LiveWindow.disableAllTelemetry();
    // CameraServer.startAutomaticCapture();
    // UsbCamera cs = CameraServer.startAutomaticCapture();
    // cs.setResolution(200, 130);
    // cs.setFPS(15);

    SmartDashboard.putBoolean("CLIMB NOW", false);
    SmartDashboard.putBoolean("Jetson Connected", false);



    // new RunCommand(() -> new IntakeCmd(m_intake).stopIntake(), m_intake);
    AAPowerDistribution.ringLightOff();
    SmartDashboard.putBoolean("Shoot Ready", false);
    

    drive.zeroSensors();


    
    // new RunCommand(() -> new LightsCmd(m_PDH).ringLightOff(), m_PDH);

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.

    

    CommandScheduler.getInstance().run();

    //Initalize Dashboard
    //Dashboard dashboard = new Dashboard();
    //Get values for dashboard and print
    //dashboard.updateVals();
    //dashboard.showVals();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    // rightLead.setNeutralMode(NeutralMode.Coast);
    // rightFollow.setNeutralMode(NeutralMode.Coast);
    // leftLead.setNeutralMode(NeutralMode.Coast);
    // leftFollow.setNeutralMode(NeutralMode.Coast);
    elevator.rightElevatorMotor.set(TalonFXControlMode.PercentOutput, 0);
    elevator.leftElevatorMotor.set(TalonFXControlMode.PercentOutput,  0);
    AAPowerDistribution.ringLightOff();

  }

  @Override
  public void disabledPeriodic() 
  {


  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {

    drive.zeroSensors();
    
    shooter.setSelectedSensorPosition(0);
    
    rightLead.setNeutralMode(NeutralMode.Brake);
    rightFollow.setNeutralMode(NeutralMode.Brake);
    leftLead.setNeutralMode(NeutralMode.Brake);
    leftFollow.setNeutralMode(NeutralMode.Brake);
    AAPowerDistribution.ringLightOn();

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    //AAPowerDistribution.ringLightOn();


    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {

    elevator.rightElevatorMotor.set(TalonFXControlMode.PercentOutput, 0);
    elevator.leftElevatorMotor.set(TalonFXControlMode.PercentOutput,  0);

    rightLead.setNeutralMode(NeutralMode.Brake);
    rightFollow.setNeutralMode(NeutralMode.Brake);
    leftLead.setNeutralMode(NeutralMode.Brake);
    leftFollow.setNeutralMode(NeutralMode.Brake);


    m_intake.stopSubsystemIntake();
    m_feeder.stopSubsystemFeed();


    // new RunCommand(() -> new IntakeCmd(m_intake).stopIntake(), m_intake);
    // new RunCommand(() -> new FeederCmd(m_feeder).stopFeeder(), m_feeder);

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if (DriverStation.getMatchTime() <= 45)
    {
      SmartDashboard.putBoolean("CLIMB NOW", true);
    }

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
