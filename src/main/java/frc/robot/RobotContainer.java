// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Joystick;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.commands.ExampleCommand;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  //init the joystick
  Joystick joystick1 = new Joystick(0);

  //buttons init
  JoystickButton button1 = new JoystickButton(joystick1, 1);
  JoystickButton button2 = new JoystickButton(joystick1, 2);
  JoystickButton button3 = new JoystickButton(joystick1, 3);


  // The robot's subsystems and commands are defined here...

  //default drive with the DoubleSupplier Lambdas
  //private final DriveSubsystem m_drive = new DriveSubsystem();
  //private final DefaultDriveCmd defaultDrive = new DefaultDriveCmd(m_drive, () -> joystick1.getY(), () -> joystick1.getZ());

  //shooter control command with PID stuff + lambdas
  private final ShooterPID m_shooter = new ShooterPID();
  private final ShooterCmd shooterControl = new ShooterCmd(m_shooter, () -> (((joystick1.getRawAxis(3) + 1) / 2) * 7500));


  //feeder command stuff
  private final Feeder m_feeder = new Feeder();
  private final FeederCmd feedControl = new FeederCmd(m_feeder);

  private final ExampleCommand exampleCommand = new ExampleCommand();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    
    //set default drive to be the default driveSubsystem command
   // m_drive.setDefaultCommand(defaultDrive);

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    button1.whileHeld(shooterControl);

    button2.whileHeld(new StartEndCommand(
      // Start a flywheel spinning at 50% power
      () -> feedControl.runFeederForward(),
      // Stop the flywheel at the end of the command
      () -> feedControl.stopFeeder(),
      // Requires the feeder subsystem
      m_feeder
    ));

    button3.whileHeld(new StartEndCommand(
      // Start a flywheel spinning at 50% power
      () -> feedControl.runFeederBackwards(),
      // Stop the flywheel at the end of the command
      () -> feedControl.stopFeeder(),
      // Requires the feeder subsystem
      m_feeder
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
