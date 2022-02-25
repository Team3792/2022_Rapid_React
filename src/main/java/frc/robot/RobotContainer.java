// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj.Joystick;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj.XboxController;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  //init the joystick
  Joystick driveJoystick = new Joystick(Constants.ButtonConstant.kDriveJoystick);

  //buttons init
  JoystickButton testShootButton = new JoystickButton(driveJoystick, 10);
  JoystickButton alertShootButton = new JoystickButton(driveJoystick, Constants.ButtonConstant.kAlertShoot);
  JoystickButton intakeButton = new JoystickButton(driveJoystick, 1);
  JoystickButton readyShoot = new JoystickButton(driveJoystick, 5);




  XboxController operateController = new XboxController(Constants.ButtonConstant.kOperateController);
  JoystickButton shootButton = new JoystickButton(operateController, 1);

  //JoystickButton testShootButton = new JoystickButton(driveJoystick, 10);


  JoystickButton feederButton = new JoystickButton(operateController, Constants.ButtonConstant.kRunFeederButton);
  JoystickButton revFeederButton = new JoystickButton(operateController, 3);
  JoystickButton revIntakeButton = new JoystickButton(operateController, 2);



  // The robot's subsystems and commands are defined here...

  //default drive with the DoubleSupplier Lambdas
  private final DriveSubsystem m_drive = new DriveSubsystem();
  //private final DefaultDriveCmd defaultDrive = new DefaultDriveCmd(m_drive, () -> driveJoystick.getY(), () -> driveJoystick.getZ());

  //shooter control command with PID stuff + lambdas
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  //private final ShooterCmd shooterControl = new ShooterCmd(m_shooter, () -> (((driveJoystick.getRawAxis(3) + 1) / 2) * 7500));


  //feeder command stuff
  private final FeedSubsystem m_feeder = new FeedSubsystem();
  private final FeederCmd feedControl = new FeederCmd(m_feeder);
  private final RumbleCmd rumble = new RumbleCmd();


  private final IntakeSubsystem m_intake = new IntakeSubsystem();

  private final ExampleCommand exampleCommand = new ExampleCommand();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    
    //set default drive to be the default driveSubsystem command


    m_drive.setDefaultCommand(new DefaultDriveCmd(m_drive, 
            () -> driveJoystick.getRawAxis(1), 
            () -> driveJoystick.getRawAxis(2))
    );


    //m_drive.setDefaultCommand(defaultDrive);
    

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    
    
    testShootButton.whileHeld(new ShooterCmd(m_shooter, 
        () -> ((driveJoystick.getRawAxis(3) + 1) / 2) * 7500));

    shootButton.whileHeld(new ShooterCmd(m_shooter, 
    () -> ((driveJoystick.getRawAxis(3) + 1) / 2) * 7500));

    readyShoot.whileHeld(new StartEndCommand(
      // Start a flywheel spinning at 50% power
      () -> rumble.startShake(),
      // Stop the flywheel at the end of the command
      () -> rumble.stopShake(),
      // Requires the feeder subsystem
      m_feeder
    ));


    feederButton.whileHeld(new StartEndCommand(
      // Start a flywheel spinning at 50% power
      () -> feedControl.runFeederForward(),
      // Stop the flywheel at the end of the command
      () -> feedControl.stopFeeder(),
      // Requires the feeder subsystem
      m_feeder
    ));

    revFeederButton.whileHeld(new StartEndCommand(
      // Start a flywheel spinning at 50% power
      () -> feedControl.runFeederBackwards(),
      // Stop the flywheel at the end of the command
      () -> feedControl.stopFeeder(),
      // Requires the feeder subsystem
      m_feeder
    ));

    intakeButton.whileHeld(new StartEndCommand(
      
      () -> new IntakeCmd(m_intake).runIntakeForward(),

      () -> new IntakeCmd(m_intake).stopIntake(),

      m_intake
    ));

    revIntakeButton.whileHeld(new StartEndCommand(
      
      () -> new IntakeCmd(m_intake).runIntakeBackward(),

      () -> new IntakeCmd(m_intake).stopIntake(),

      m_intake
    ));

    






  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  //public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    //return m_autoCommand;
  //}
  public Command getAutonomousCommand(){
    //filler for rn- change later when we actually have an auto command group
    return exampleCommand;
  }
}
