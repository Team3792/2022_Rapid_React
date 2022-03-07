// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Joystick.*;
import frc.robot.commands.*;
import frc.robot.commands.AutoRoutines.*;


import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj.Joystick;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.SemiAuto.semiAutoAlignCmd;
import frc.robot.Joystick.PS5Mapping;




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
  JoystickButton intakeButton = new JoystickButton(driveJoystick, 1);
  JoystickButton readyShoot = new JoystickButton(driveJoystick, 5);
  JoystickButton targetAlign = new JoystickButton(driveJoystick, Constants.ButtonConstant.targetAlignButton);
  JoystickButton ballAlign = new JoystickButton(driveJoystick, Constants.ButtonConstant.ballAlignButton);



  PS5Mapping operateController = new PS5Mapping();




  //Auto Choose Sendable Class
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();


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
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  private final ClimbSubsystem m_climber = new ClimbSubsystem();
  private final ServoSubsystem m_servo = new ServoSubsystem();


  //Auto Declarators
  private final Auto2Ball auto2ball = new Auto2Ball(m_drive, 
  m_intake, m_feeder, m_shooter);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    
    //set default drive to be the default driveSubsystem command


    m_drive.setDefaultCommand(new DefaultDriveCmd(m_drive, 
            () -> driveJoystick.getRawAxis(1), 
            () -> driveJoystick.getRawAxis(2))
    );

    autoChooser.setDefaultOption("2 Ball Auto", auto2ball);



    //m_drive.setDefaultCommand(defaultDrive);
    

  }
  

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    
    


    // operateController.XOnlyButton.whileHeld(new ShooterCmd(m_shooter, 
    // () -> ((driveJoystick.getRawAxis(3) + 1) / 2) * 7500, false));

    operateController.XOnlyButton.whileHeld(new ShooterCmd(m_shooter, 5000, false));



    targetAlign.whileHeld(new semiAutoAlignCmd(m_drive, () -> driveJoystick.getY(), 0));


    // targetAlign.whileHeld(new StartEndCommand(
      
    // () -> new LightsCmd(m_PDH).ringLightOn(),
    
    // () -> new LightsCmd(m_PDH).ringLightOff(), 
    
    // m_PDH));

    ballAlign.whileHeld(new semiAutoAlignCmd(m_drive, () -> driveJoystick.getY(), 1));

    




//Elevator

operateController.CircleOnlyButton.whenPressed(new ParallelCommandGroup(

new InstantCommand(m_elevator::zeroSensors, m_elevator),

new InstantCommand(m_climber::zeroSensors, m_climber)

));

operateController.climbUp.and(operateController.SquareOnlyButton).whenActive(new ElevatorCmd(
  
m_elevator, 

Constants.ElevatorConstants.setpointUp

));

operateController.climbDown.and(operateController.SquareOnlyButton).whenActive(new ElevatorCmd(
  
m_elevator, 

Constants.ElevatorConstants.setpointDown

));


operateController.LStickButton.and(operateController.climbUp).whenActive(new InstantCommand(

m_elevator::moveElevatorUp,

m_elevator

));

operateController.LStickButton.and(operateController.climbDown).whenActive(new InstantCommand(

m_elevator::moveElevatorDown,

m_elevator

));

operateController.LStickButton.whenReleased(new InstantCommand(

m_elevator::stopElevator,

m_elevator

));

operateController.TriangleOnlyButton.whileHeld(new StartEndCommand(

m_climber::moveLeftUp,

m_climber::stopClimb,

m_elevator

));

//Pivot, Climb, idk whatever we're calling it

operateController.pivotForward.and(operateController.SquareOnlyButton).whenActive(new ClimbCmd(
  
m_climber, 

Constants.ClimbConstants.setpointForward

));

operateController.pivotBack.and(operateController.SquareOnlyButton).whenActive(new ClimbCmd(

m_climber, 

Constants.ClimbConstants.setpointBack

));


operateController.RStickButton.and(operateController.pivotForward).whenActive(new InstantCommand(

m_climber::moveClimbForward,

m_climber

));

operateController.RStickButton.and(operateController.pivotBack).whenActive(new InstantCommand(

m_climber::moveClimbBack,

m_climber

));

operateController.RStickButton.whenReleased(new InstantCommand(

m_climber::stopClimb,

m_climber

));

readyShoot.whileHeld(new StartEndCommand(
  // Start a flywheel spinning at 50% power
  () -> operateController.startShake(),
  // Stop the flywheel at the end of the command
  () -> operateController.stopShake(),
  // Requires the feeder subsystem
  m_feeder

));


operateController.RTriggerButton.whileHeld(new StartEndCommand(
  // Start a flywheel spinning at 50% power
  () -> feedControl.runFeederForward(),
  // Stop the flywheel at the end of the command
  () -> feedControl.stopFeeder(),
  // Requires the feeder subsystem
  m_feeder

));

operateController.POVLeftish.whileActiveContinuous(new StartEndCommand(
  // Start a flywheel spinning at 50% power
  () -> feedControl.runFeederBackwards(),
  // Stop the flywheel at the end of the command
  () -> feedControl.stopFeeder(),
  // Requires the feeder subsystem
  m_feeder

));




operateController.POVUpish.whileActiveContinuous(new StartEndCommand(
  
  () -> new IntakeCmd(m_intake).runIntakeForwardSlow(),

  () -> new IntakeCmd(m_intake).stopIntake(),

  m_intake

));


intakeButton.whileHeld(new StartEndCommand(
  
  () -> new IntakeCmd(m_intake).runIntakeForward(),

  () -> new IntakeCmd(m_intake).stopIntake(),

  m_intake
  
));

operateController.POVDownish.whileActiveContinuous(new StartEndCommand(
  
  () -> new IntakeCmd(m_intake).runIntakeBackward(),

  () -> new IntakeCmd(m_intake).stopIntake(),

  m_intake

));

operateController.POVRightish.whileActiveContinuous(new StartEndCommand(

  () -> new SetShootCmd(m_shooter).reverseShooter(), 

  () -> new SetShootCmd(m_shooter).stopShoot(), 
  
  m_shooter
  
));

operateController.LFaceButton.whenPressed(new InstantCommand(
  
  AAPowerDistribution::ringLightOn

));

operateController.RFaceButton.whenPressed(new InstantCommand(
    
  AAPowerDistribution::ringLightOff

));

operateController.L1Button.whileHeld(new RunCommand(
    () -> new ServoCmd(m_servo).openServo(),

    m_servo



));

operateController.R1Button.whileHeld(new RunCommand(
    () -> new ServoCmd(m_servo).closeServo(),
    
    m_servo


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
    return new Auto2Ball(m_drive, m_intake, m_feeder, m_shooter);
  }
}
